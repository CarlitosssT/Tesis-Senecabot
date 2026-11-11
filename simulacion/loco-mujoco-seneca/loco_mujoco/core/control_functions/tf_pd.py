"""
[SENECA LOCAL CHANGE] Position control through an identified actuator transfer function (see SENECA_LOCAL_CHANGES.md,
2026-11-11).

The policy commands the angle of every actuated joint. Each command goes through the transfer function identified on
the real MyActuator in motion mode 0x400 (hardware/myactuator_esp32/sysid/, tf_por_modo.json):

    G(s) = K / (tau s + 1) * exp(-Td s)          (default K = 1.0130, tau = 14.61 ms, Td = 5.34 ms)

and a stiff PD, saturated at the actuator ctrlrange (the motor torque limits of the XML), makes the joint follow the
filtered angle. With the PD much faster than tau the joint reproduces G(s) at no load; under load the torque limit
slows it down, as on the real motor.

Discretisation, at the simulation step dt (the controller runs with the simulation frequency):
  * delay: ring buffer of past commands with linear interpolation for the fractional part of Td / dt;
  * first order: exact zero-order-hold step, y[k+1] = a y[k] + (1 - a) (c + K (u_d[k] - c)), a = exp(-dt / tau);
    the gain K acts on the displacement from the actuator's center c (assumed mounting zero of the motor: the bench
    identification started and ended at 0 deg), not on the absolute joint angle;
  * PD: torque = kp (y - q) - kd qdot, clipped to ctrlrange.

Unlike PDControl, the action is not scaled to the joint range (the RR crank joints are unlimited): action a in [-1, 1]
maps to  center + a * half_range  per actuator (radians; given in degrees in the config).
At reset the filter and the delay buffer are set to the current joint angle, so the actuators start at rest.
"""
from typing import Any, List, Tuple, Union
from types import ModuleType

import numpy as np
import jax
import mujoco
from flax import struct
from mujoco import MjData, MjModel
from mujoco.mjx import Data, Model

from loco_mujoco.core.utils.backend import assert_backend_is_supported
from loco_mujoco.core.control_functions import ControlFunction
from loco_mujoco.core.utils import mj_jntname2qposid, mj_jntname2qvelid


@struct.dataclass
class TFPDControlState:
    filt: Union[np.ndarray, jax.Array]        # output of G(s): the angle the PD tracks [rad], shape (n_act,)
    delay_buf: Union[np.ndarray, jax.Array]   # past commands, newest first [rad], shape (n_buf, n_act)


class TFPDControl(ControlFunction):

    def __init__(self, env: Any,
                 p_gain: Union[float, List[float]],
                 d_gain: Union[float, List[float]],
                 action_center_deg: List[float],
                 action_half_range_deg: List[float],
                 tf_gain: float = 1.0129645664540181,
                 tf_tau: float = 0.014612171384488929,
                 tf_delay: float = 0.005340280763591637,
                 **kwargs: Any):
        """
        Args:
            env: environment instance.
            p_gain, d_gain: PD gains [N.m/rad], [N.m.s/rad]; scalar or one per actuator (action order).
            action_center_deg, action_half_range_deg: one per actuator (action order); action a in [-1, 1] commands
                center + a * half_range.
            tf_gain, tf_tau [s], tf_delay [s]: K, tau and Td of G(s). tf_tau <= 0 disables the lag, tf_delay = 0 the
                delay (tf_gain = 1, tf_tau = 0, tf_delay = 0 is a plain PD on the commanded angle).
        """
        n = len(env._action_indices)
        self._jnt_names, ctrl_ranges = [], []
        for i in env._action_indices:
            act = env.mjspec.actuators[i]
            for j in env.mjspec.joints:
                if j.name == act.target:
                    assert j.type in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE), \
                        "Only hinge and slide joints are supported by TFPDControl."
            self._jnt_names.append(act.target)
            ctrl_ranges.append(act.ctrlrange if act.ctrllimited else np.array([-np.inf, np.inf]))
        self._ctrl_ranges = np.array(ctrl_ranges, dtype=float)
        self._qpos_ids = np.concatenate([mj_jntname2qposid(name, env._model) for name in self._jnt_names])
        self._qvel_ids = np.concatenate([mj_jntname2qvelid(name, env._model) for name in self._jnt_names])

        def per_actuator(v, name):
            v = np.broadcast_to(np.asarray(v, dtype=float), (n,)).copy()
            assert v.shape == (n,), f"{name}: expected {n} values"
            return v
        self._p_gain = per_actuator(p_gain, "p_gain")
        self._d_gain = per_actuator(d_gain, "d_gain")
        self._center = np.deg2rad(per_actuator(action_center_deg, "action_center_deg"))
        self._half_range = np.deg2rad(per_actuator(action_half_range_deg, "action_half_range_deg"))

        # the controller is called before every physics step (run_with_simulation_frequency)
        self._dt = float(env._model.opt.timestep)
        self._gain = float(tf_gain)
        self._alpha = float(np.exp(-self._dt / tf_tau)) if tf_tau > 0 else 0.0
        d = max(float(tf_delay), 0.0) / self._dt
        self._delay_int = int(np.floor(d))
        self._delay_frac = d - self._delay_int
        self._n_buf = self._delay_int + 2

        super().__init__(env, -np.ones(n), np.ones(n), **kwargs)

    @property
    def run_with_simulation_frequency(self):
        return True

    def init_state(self, env: Any, key: Any, model: Union[MjModel, Model], data: Union[MjData, Data],
                   backend: ModuleType) -> TFPDControlState:
        assert_backend_is_supported(backend)
        q = data.qpos[self._qpos_ids]
        return TFPDControlState(filt=q, delay_buf=backend.tile(self._at_rest_command(q), (self._n_buf, 1)))

    def reset(self, env: Any, model: Union[MjModel, Model], data: Union[MjData, Data], carry: Any,
              backend: ModuleType) -> Tuple[Union[MjData, Data], Any]:
        """Start at rest on the reset pose (called after the initial state handler set qpos)."""
        assert_backend_is_supported(backend)
        q = data.qpos[self._qpos_ids]
        state = TFPDControlState(filt=q, delay_buf=backend.tile(self._at_rest_command(q), (self._n_buf, 1)))
        return data, carry.replace(control_func_state=state)

    def _at_rest_command(self, q):
        """Command whose steady-state output of G(s) is q."""
        return self._center + (q - self._center) / self._gain

    def generate_action(self, env: Any, action: Union[np.ndarray, jax.Array], model: Union[MjModel, Model],
                        data: Union[MjData, Data], carry: Any,
                        backend: ModuleType) -> Tuple[Union[np.ndarray, jax.Array], Any]:
        assert_backend_is_supported(backend)
        state = carry.control_func_state
        command = self._center + action * self._half_range                        # commanded angle [rad]

        # delay: newest command first, interpolate between the two samples around Td
        buf = backend.concatenate([command[None, :], state.delay_buf[:-1]], axis=0)
        delayed = ((1.0 - self._delay_frac) * buf[self._delay_int]
                   + self._delay_frac * buf[self._delay_int + 1])

        # first order lag (exact for a command held over dt), gain K
        target = self._center + self._gain * (delayed - self._center)
        filt = self._alpha * state.filt + (1.0 - self._alpha) * target

        # stiff PD on the filtered angle, saturated at the motor torque limits
        torque = (self._p_gain * (filt - data.qpos[self._qpos_ids])
                  - self._d_gain * data.qvel[self._qvel_ids])
        torque = backend.clip(torque, self._ctrl_ranges[:, 0], self._ctrl_ranges[:, 1])

        carry = carry.replace(control_func_state=TFPDControlState(filt=filt, delay_buf=buf))
        return torque, carry
