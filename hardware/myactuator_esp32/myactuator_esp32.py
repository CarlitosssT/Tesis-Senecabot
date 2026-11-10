"""Libreria para mover un motor MyActuator por CAN a traves de una ESP32.

La ESP32 corre Arduino/motor_control/motor_control.ino (en PCR_GP-main) y traduce
comandos de texto a tramas CAN. Esta libreria solo necesita pyserial.

Uso basico:
    from myactuator_esp32 import MyActuator

    with MyActuator("COM3", motor_id=1) as motor:
        motor.move_to(30, max_speed_dps=60)       # va a 30 grados y espera
        print(motor.read_angle())                 # angulo con resolucion de 0.01 grados
        log = motor.play(times_s, angles_deg)     # trayectoria en tiempo real
        log.to_csv("registro.csv")

En play() la trayectoria se sube a la ESP32 y la reproduce ella con su reloj, de modo
que los tiempos no dependen del PC ni del USB. Mientras corre, la ESP32 lee el
encoder del motor (0x92) tras cada comando y guarda el angulo medido.
"""
import csv
import math
import re
import time
from dataclasses import dataclass, field

import serial

# Mismos limites que motor_control.ino
MAX_ANGLE_DEG = 360.0
MAX_SPEED_DPS = 300       # comandos p y v
MAX_RUN_SPEED_DPS = 600   # limite de velocidad al reproducir una trayectoria
MAX_POINTS = 1000

# Aceleraciones del motor (comandos 0x42 / 0x43), en dps/s
MIN_ACCEL_DPS2 = 100
MAX_ACCEL_DPS2 = 60000
ACCEL_NAMES = ("position_accel", "position_decel", "speed_accel", "speed_decel")

# Modos de control al reproducir una trayectoria
MODE_POSITION = 0   # 0xA4: el planificador del motor sigue cada posicion
MODE_MOTION = 1     # 0x400: lazo kp/kd dentro del motor con velocidad de referencia
MM_KP_MAX = 500.0   # N.m/rad
MM_KD_MAX = 5.0     # N.m/(rad/s)
MM_VFF_MAX = 2.0

# Torque (0xA1), mismos limites que motor_control.ino
MAX_TORQUE_A = 5.0
MIN_TORQUE_MS = 50
MAX_TORQUE_MS = 10000

INT32_MIN = -(2 ** 31)
_STATUS_RE = re.compile(
    r"Temp:\s*(-?\d+)\s*C.*Corriente:\s*(-?[\d.]+)\s*A.*Velocidad:\s*(-?\d+)\s*dps"
    r".*Angulo:\s*(-?\d+)")


_OLD_FIRMWARE = ("la ESP32 no responde a los comandos de aceleracion: sube la version "
                 "actual de motor_control.ino")


_OLD_FIRMWARE_MODE = ("la ESP32 no responde: sube la version actual de motor_control.ino "
                      "(posicion 0x400 y torque)")


class MotorError(RuntimeError):
    """El motor o la ESP32 respondieron con un error."""


@dataclass
class TrajectoryLog:
    """Registro de una ejecucion. Los tiempos son segundos desde el inicio."""
    t_cmd_s: list = field(default_factory=list)   # instante en que se envio cada comando
    cmd_deg: list = field(default_factory=list)   # angulo enviado
    t_meas_s: list = field(default_factory=list)  # instante en que se leyo el encoder
    meas_deg: list = field(default_factory=list)  # angulo medido (nan si no hubo lectura)

    @property
    def missed(self):
        return sum(1 for m in self.meas_deg if math.isnan(m))

    def to_csv(self, path):
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["t_cmd_s", "cmd_deg", "t_meas_s", "meas_deg"])
            for row in zip(self.t_cmd_s, self.cmd_deg, self.t_meas_s, self.meas_deg):
                writer.writerow(row)


def peak_speed_dps(times_s, angles_deg):
    """Velocidad maxima que exige la trayectoria entre puntos consecutivos."""
    return max(abs(angles_deg[k + 1] - angles_deg[k]) / (times_s[k + 1] - times_s[k])
               for k in range(len(times_s) - 1))


class MyActuator:
    def __init__(self, port=None, baud=115200, motor_id=1, boot_wait=2.5, timeout=1.0,
                 ser=None):
        """port: puerto de la ESP32 (COM3...). ser: puerto ya abierto, para pruebas."""
        self.port = port
        self.baud = baud
        self.motor_id = motor_id
        self.boot_wait = boot_wait
        self.timeout = timeout
        self.ser = ser
        self.control_mode = MODE_POSITION

    # --- conexion ---------------------------------------------------------

    def open(self):
        if self.ser is None:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(self.boot_wait)  # la ESP32 se reinicia al abrir el puerto
        self.ser.reset_input_buffer()
        self._send(f"m {self.motor_id}")
        self._wait_for("Motor seleccionado")
        self.control_mode = MODE_POSITION  # la ESP32 arranca en modo 0xA4
        return self

    def close(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, tb):
        try:
            if self.ser is not None:
                self.stop()
        finally:
            self.close()

    # --- comunicacion de bajo nivel ----------------------------------------

    def _send(self, command):
        self.ser.write((command + "\n").encode("ascii"))

    def _wait_for(self, prefix, timeout=None):
        """Lee lineas hasta una que empiece por prefix; ignora el resto."""
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            line = self.ser.readline().decode("ascii", errors="replace").strip()
            if line.startswith("ERR"):
                raise MotorError(line)
            if line.startswith(prefix):
                return line
        raise TimeoutError(f"sin respuesta {prefix!r} de la ESP32")

    # --- comandos simples ---------------------------------------------------

    def set_position(self, angle_deg, max_speed_dps=100):
        """Envia una posicion absoluta sin esperar a que llegue."""
        if abs(angle_deg) > MAX_ANGLE_DEG:
            raise ValueError(f"angulo {angle_deg} fuera de +-{MAX_ANGLE_DEG:g} grados")
        if not 1 <= max_speed_dps <= MAX_SPEED_DPS:
            raise ValueError(f"velocidad {max_speed_dps} fuera de 1-{MAX_SPEED_DPS} dps")
        self._send(f"p {angle_deg:g} {int(max_speed_dps)}")

    def set_speed(self, dps):
        if abs(dps) > MAX_SPEED_DPS:
            raise ValueError(f"velocidad {dps} fuera de +-{MAX_SPEED_DPS} dps")
        self._send(f"v {dps:g}")

    def stop(self):
        self._send("s")

    def read_angle(self):
        """Angulo multivuelta del eje de salida en grados, con resolucion de 0.01."""
        self.ser.reset_input_buffer()
        self._send("a")
        line = self._wait_for("ANG")
        value = line.split()[1]
        if value == "TIMEOUT":
            raise MotorError("el motor no responde (revisa el CAN y la alimentacion)")
        return int(value) / 100.0

    def read_status(self):
        """Temperatura (C), corriente (A), velocidad (dps) y angulo entero (grados)."""
        self.ser.reset_input_buffer()
        self._send("e")
        line = self._wait_for("Motor")
        match = _STATUS_RE.search(line)
        if not match:
            raise MotorError(f"respuesta de estado no reconocida: {line!r}")
        temp, current, speed, angle = match.groups()
        return {"temperature_c": int(temp), "current_a": float(current),
                "speed_dps": int(speed), "angle_deg": int(angle)}

    def read_accelerations(self):
        """Aceleraciones del motor en dps/s: {position_accel, position_decel,
        speed_accel, speed_decel}. Las de posicion son las que planifica el 0xA4."""
        self.ser.reset_input_buffer()
        self._send("Q")
        values = {}
        for index, name in enumerate(ACCEL_NAMES):
            try:
                line = self._wait_for("ACC", timeout=self.timeout + 0.5)
            except TimeoutError:
                raise MotorError(_OLD_FIRMWARE) from None
            value = line.split()[2]
            if value == "TIMEOUT":
                raise MotorError(f"el motor no respondio a la lectura de {name}")
            values[name] = int(value)
        return values

    def write_accel(self, index, accel_dps2):
        """Escribe una aceleracion (indice 0-3, ver ACCEL_NAMES). Se guarda en la ROM del
        motor; hazlo con el motor parado."""
        if index not in range(len(ACCEL_NAMES)):
            raise ValueError(f"indice {index} fuera de 0-{len(ACCEL_NAMES) - 1}")
        if not MIN_ACCEL_DPS2 <= accel_dps2 <= MAX_ACCEL_DPS2:
            raise ValueError(
                f"aceleracion {accel_dps2} fuera de {MIN_ACCEL_DPS2}-{MAX_ACCEL_DPS2} dps/s")
        self.ser.reset_input_buffer()
        self._send(f"W {index} {int(accel_dps2)}")
        try:
            line = self._wait_for("ACC", timeout=self.timeout + 0.5)
        except TimeoutError:
            raise MotorError(_OLD_FIRMWARE) from None
        if line.split()[2] == "TIMEOUT":
            raise MotorError(f"el motor no confirmo la escritura de {ACCEL_NAMES[index]}")

    def set_control_mode(self, mode, kp=20.0, kd=1.0, vff=1.0):
        """Elige como se sigue la trayectoria de play()/run_trajectory().

        MODE_POSITION (0): comando 0xA4, el planificador del motor.
        MODE_MOTION (1): comando 0x400, par = kp*(p_des - p) + kd*(v_des - v). La velocidad
        de referencia v_des es la pendiente de la trayectoria por vff (feedforward).
        kp en N.m/rad (0-500), kd en N.m/(rad/s) (0-5). Los movimientos manuales
        (set_position, move_to) siguen usando 0xA4. El ajuste vive en la RAM de la ESP32 y
        se pierde al reiniciarla.
        """
        if mode not in (MODE_POSITION, MODE_MOTION):
            raise ValueError("modo debe ser 0 (posicion 0xA4) o 1 (movimiento 0x400)")
        if not 0 <= kp <= MM_KP_MAX:
            raise ValueError(f"kp {kp} fuera de 0-{MM_KP_MAX:g}")
        if not 0 <= kd <= MM_KD_MAX:
            raise ValueError(f"kd {kd} fuera de 0-{MM_KD_MAX:g}")
        if not 0 <= vff <= MM_VFF_MAX:
            raise ValueError(f"vff {vff} fuera de 0-{MM_VFF_MAX:g}")
        self.ser.reset_input_buffer()
        self._send(f"M {mode} {kp:g} {kd:g} {vff:g}")
        try:
            self._wait_for("OK MODE")
        except TimeoutError:
            raise MotorError("la ESP32 no responde al cambio de modo: sube la version "
                             "actual de motor_control.ino") from None
        self.control_mode = mode

    def set_motion_position(self, angle_deg):
        """Posicion con el modo 0x400 (usa kp/kd de set_control_mode). No hay planificador
        ni limite de velocidad: un salto grande da un tiron."""
        if abs(angle_deg) > MAX_ANGLE_DEG:
            raise ValueError(f"angulo {angle_deg} fuera de +-{MAX_ANGLE_DEG:g} grados")
        self.ser.reset_input_buffer()
        self._send(f"P {angle_deg:g}")
        try:
            self._wait_for("OK MPOS")
        except TimeoutError:
            raise MotorError(_OLD_FIRMWARE_MODE) from None

    def set_torque(self, current_a, duration_ms=2000):
        """Corriente de par (0xA1) en amperios durante duration_ms; despues la ESP32 para el
        motor (0x81). Sin limite de velocidad: con el eje libre acelera mientras dure.
        Para antes con stop()."""
        if abs(current_a) > MAX_TORQUE_A:
            raise ValueError(f"corriente {current_a} fuera de +-{MAX_TORQUE_A:g} A")
        if not MIN_TORQUE_MS <= duration_ms <= MAX_TORQUE_MS:
            raise ValueError(f"duracion {duration_ms} fuera de {MIN_TORQUE_MS}-{MAX_TORQUE_MS} ms")
        self.ser.reset_input_buffer()
        self._send(f"i {current_a:g} {int(duration_ms)}")
        try:
            self._wait_for("OK TORQUE")
        except TimeoutError:
            raise MotorError(_OLD_FIRMWARE_MODE) from None

    def motion_mode_test(self):
        """Prueba el modo 0x400 con ganancias bajas: pide +5 grados, espera 0.5 s y vuelve.
        Devuelve (angulo_inicial, angulo_final) en grados; si el modo 0x400 usa el mismo
        cero y sentido que el encoder, el cambio es de unos +5."""
        self.ser.reset_input_buffer()
        self._send("T")
        try:
            line = self._wait_for("MT", timeout=self.timeout + 1.5)
        except TimeoutError:
            raise MotorError("la ESP32 no responde a la prueba: sube la version actual de "
                             "motor_control.ino") from None
        parts = line.split()
        if parts[1] == "TIMEOUT":
            raise MotorError("el motor no respondio a la prueba del modo 0x400")
        return int(parts[1]) / 100.0, int(parts[2]) / 100.0

    def move_to(self, angle_deg, max_speed_dps=60, tolerance_deg=0.5, poll_s=0.05):
        """Va a una posicion y espera a llegar. Devuelve el angulo final."""
        start = self.read_angle()
        self.set_position(angle_deg, max_speed_dps)
        timeout = abs(angle_deg - start) / max_speed_dps * 1.5 + 3.0
        deadline = time.monotonic() + timeout
        angle = start
        while time.monotonic() < deadline:
            angle = self.read_angle()
            if abs(angle - angle_deg) <= tolerance_deg:
                return angle
            time.sleep(poll_s)
        raise TimeoutError(
            f"no llego a {angle_deg:g} grados en {timeout:.1f} s (ultimo angulo: {angle})")

    # --- trayectorias --------------------------------------------------------

    def upload_trajectory(self, times_s, angles_deg):
        """Sube la tabla a la ESP32. El primer instante se toma como t = 0."""
        n = len(times_s)
        if n != len(angles_deg) or not 2 <= n <= MAX_POINTS:
            raise ValueError(f"se necesitan entre 2 y {MAX_POINTS} puntos con tiempo y angulo")
        if any(b <= a for a, b in zip(times_s, times_s[1:])):
            raise ValueError("los tiempos deben ser estrictamente crecientes")
        if any(abs(a) > MAX_ANGLE_DEG for a in angles_deg):
            raise ValueError(f"hay angulos fuera de +-{MAX_ANGLE_DEG:g} grados")
        t0 = times_s[0]
        self.ser.reset_input_buffer()
        self._send(f"L {n}")
        self._wait_for("OK LOAD")
        for t, a in zip(times_s, angles_deg):
            self._send(f"{round((t - t0) * 1e6)} {round(a * 100)}")
        self._wait_for("OK LOADED", timeout=self.timeout + n * 0.01)

    def abort(self):
        """Aborta una trayectoria en curso. Se puede llamar desde otro hilo."""
        self._send("x")

    def run_trajectory(self, max_speed_dps, duration_s):
        """Reproduce la tabla subida. Con Ctrl+C aborta y para el motor."""
        speed = int(min(max_speed_dps, MAX_RUN_SPEED_DPS))
        self._send(f"R {speed}")
        self._wait_for("OK RUN")
        try:
            line = self._wait_for("OK", timeout=duration_s + 5.0)
        except KeyboardInterrupt:
            self._send("x")
            self._wait_for("OK", timeout=2.0)
            self.stop()
            raise
        if not line.startswith(("OK DONE", "OK ABORT")):
            raise MotorError(f"fin de ejecucion inesperado: {line!r}")
        return line

    def download_log(self):
        self.ser.reset_input_buffer()
        self._send("D")
        header = self._wait_for("LOG")
        n = int(header.split()[1])
        log = TrajectoryLog()
        for _ in range(n):
            line = self.ser.readline().decode("ascii", errors="replace").strip()
            t_cmd, cmd, t_meas, meas = (int(x) for x in line.split(","))
            log.t_cmd_s.append(t_cmd / 1e6)
            log.cmd_deg.append(cmd / 100.0)
            log.t_meas_s.append(t_meas / 1e6)
            log.meas_deg.append(math.nan if meas == INT32_MIN else meas / 100.0)
        self._wait_for("END")
        return log

    def play(self, times_s, angles_deg, max_speed_dps=MAX_RUN_SPEED_DPS, goto_start=True,
             start_speed_dps=60):
        """Sube y reproduce una trayectoria en tiempo real y devuelve el registro."""
        if goto_start:
            self.move_to(angles_deg[0], start_speed_dps)
        self.upload_trajectory(times_s, angles_deg)
        self.run_trajectory(max_speed_dps, times_s[-1] - times_s[0])
        return self.download_log()
