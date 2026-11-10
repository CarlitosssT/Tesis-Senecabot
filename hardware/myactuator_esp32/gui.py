"""Interfaz grafica para controlar la posicion de un MyActuator a traves de la ESP32.

    python gui.py

- Envia posiciones manuales y muestra en vivo la posicion deseada contra la medida
  (encoder, 0.01 grados) con su error.
- Carga un CSV (primera columna: tiempo en s; el resto: angulos en grados), elige una
  columna y la reproduce en tiempo real. Mientras la ESP32 reproduce la trayectoria no
  envia lecturas en vivo; al terminar se descarga el registro y se muestra en la
  pestana "Ultima ejecucion".
- Exporta a CSV los datos en vivo y el registro de la ultima ejecucion.

Necesita el sketch Arduino/motor_control/motor_control.ino en la ESP32.
"""
import csv
import math
import queue
import threading
import time
import tkinter as tk
from bisect import bisect_left
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import numpy as np
import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from serial.tools import list_ports

import tracking
from myactuator_esp32 import (ACCEL_NAMES, MAX_ACCEL_DPS2, MAX_ANGLE_DEG, MAX_POINTS,
                              MAX_RUN_SPEED_DPS, MAX_SPEED_DPS, MAX_TORQUE_A, MAX_TORQUE_MS,
                              MIN_ACCEL_DPS2, MIN_TORQUE_MS, MM_KD_MAX,
                              MM_KP_MAX, MM_VFF_MAX, MODE_MOTION, MODE_POSITION, MotorError,
                              MyActuator, peak_speed_dps)

POLL_S = 0.03       # intervalo entre lecturas del encoder en vivo
EVENT_MS = 40       # cada cuanto la ventana revisa los eventos del hilo de comunicacion
REDRAW_S = 0.2      # intervalo minimo entre redibujos de la grafica en vivo (5 por segundo)
ARRIVE_TOL_DEG = 0.5


def default_motor_factory(port, motor_id):
    return MyActuator(port, motor_id=motor_id)


class Worker(threading.Thread):
    """Unico hilo que usa el puerto serie: atiende comandos y lee el encoder en vivo."""

    def __init__(self, events, motor_factory):
        super().__init__(daemon=True)
        self.events = events
        self.motor_factory = motor_factory
        self.cmds = queue.Queue()
        self.motor = None
        self.setpoint = math.nan
        self.t0 = time.monotonic()
        self.poll_failures = 0
        self._quit = threading.Event()
        self._abort = threading.Event()

    def run(self):
        while not self._quit.is_set():
            started = time.monotonic()
            try:
                while True:
                    try:
                        name, args = self.cmds.get_nowait()
                    except queue.Empty:
                        break
                    getattr(self, "_do_" + name)(*args)
                if self.motor is not None:
                    self._poll()
            except (MotorError, TimeoutError, ValueError) as exc:  # TimeoutError es un OSError
                self.events.put(("error", str(exc)))
            except (serial.SerialException, OSError) as exc:
                self.events.put(("error", f"puerto serie: {exc}"))
                self._disconnect("se perdio la conexion")
            rest = (0.3 if self.poll_failures else POLL_S) - (time.monotonic() - started)
            if rest > 0:
                self._quit.wait(rest)

    def shutdown(self):
        self._quit.set()
        self.join(3.0)
        self._disconnect("cerrado")

    def abort(self):
        """Se llama desde la ventana, sin pasar por la cola."""
        self._abort.set()
        if self.motor is not None:
            self.motor.abort()

    # --- lectura en vivo -----------------------------------------------------

    def _poll(self):
        """Lee el encoder y publica la muestra. Devuelve el angulo o None."""
        try:
            angle = self.motor.read_angle()
        except (MotorError, TimeoutError) as exc:
            self.poll_failures += 1
            if self.poll_failures == 1:
                self.events.put(("error", f"sin lectura del motor: {exc}"))
            return None
        if self.poll_failures:
            self.poll_failures = 0
            self.events.put(("status", "lectura del motor recuperada"))
        self.events.put(("sample", time.monotonic() - self.t0, self.setpoint, angle))
        return angle

    def _require_motor(self):
        if self.motor is None:
            raise MotorError("no hay conexion con la ESP32")
        return self.motor

    # --- comandos (se ejecutan en este hilo) ------------------------------------

    def _do_connect(self, port, motor_id):
        if self.motor is not None:
            self._disconnect("reconectando")
        motor = self.motor_factory(port, motor_id)
        try:
            motor.open()
            motor.read_angle()  # comprueba que el motor responde por CAN
        except Exception:
            motor.close()
            raise
        self.motor = motor
        self.t0 = time.monotonic()
        self.setpoint = math.nan
        self.poll_failures = 0
        self.events.put(("connected", port, motor_id))

    def _do_disconnect(self):
        self._disconnect("desconectado")

    def _disconnect(self, reason):
        motor, self.motor = self.motor, None
        if motor is None:
            return
        try:
            motor.stop()
        except (serial.SerialException, OSError):
            pass
        motor.close()
        self.events.put(("disconnected", reason))

    def _do_position(self, angle, speed, mode_cfg):
        motor = self._require_motor()
        if mode_cfg[0] == MODE_MOTION:
            motor.set_control_mode(*mode_cfg)
            motor.set_motion_position(angle)
        else:
            motor.set_position(angle, speed)
        self.setpoint = angle

    def _do_torque(self, current, duration_ms):
        self._require_motor().set_torque(current, duration_ms)
        self.setpoint = math.nan

    def _do_stop(self):
        self._require_motor().stop()
        self.setpoint = math.nan

    def _do_read_accel(self):
        self.events.put(("accel", self._require_motor().read_accelerations()))

    def _do_write_accel(self, values):
        motor = self._require_motor()
        for index, value in values.items():
            motor.write_accel(index, value)
        self.events.put(("status", "aceleracion escrita en el motor"))
        self._do_read_accel()

    def _do_motion_test(self):
        start, end = self._require_motor().motion_mode_test()
        self.events.put(("motion_test", start, end))

    def _do_play(self, times, angles, max_speed, start_speed, mode_cfg):
        self._abort.clear()
        self.events.put(("run_started",))
        try:
            self._require_motor().set_control_mode(*mode_cfg)
            self._play(times, angles, max_speed, start_speed)
        except (MotorError, TimeoutError, ValueError) as exc:
            self.events.put(("run_failed", str(exc)))
        except (serial.SerialException, OSError) as exc:
            self.events.put(("run_failed", f"puerto serie: {exc}"))
            self._disconnect("se perdio la conexion")

    def _play(self, times, angles, max_speed, start_speed):
        motor = self._require_motor()
        start = angles[0]
        self.setpoint = start
        motor.set_position(start, start_speed)
        angle = self._poll()
        distance = abs(start - angle) if angle is not None else MAX_ANGLE_DEG
        deadline = time.monotonic() + distance / start_speed * 1.5 + 3.0
        while angle is None or abs(angle - start) > ARRIVE_TOL_DEG:
            if self._abort.is_set():
                motor.stop()
                self.setpoint = math.nan
                raise MotorError("abortado antes de empezar la trayectoria")
            if time.monotonic() > deadline:
                raise TimeoutError(f"no llego a {start:g} grados (ultimo angulo: {angle})")
            self._quit.wait(POLL_S)
            angle = self._poll()
        motor.upload_trajectory(times, angles)
        self.events.put(("status", "ejecutando trayectoria (lecturas en vivo en pausa)"))
        motor.run_trajectory(max_speed, times[-1] - times[0])
        log = motor.download_log()
        if self._abort.is_set():
            motor.stop()
            self.setpoint = math.nan
        else:
            self.setpoint = angles[-1]
        self.events.put(("run_done", log))


class App:
    def __init__(self, root, motor_factory=default_motor_factory):
        self.root = root
        root.title("MyActuator X4-36: control de posicion")
        width = min(1280, root.winfo_screenwidth() - 80)
        height = min(800, root.winfo_screenheight() - 120)
        root.geometry(f"{width}x{height}+20+20")
        try:  # maximizada para que quepan todos los paneles (solo Windows)
            root.state("zoomed")
        except tk.TclError:
            pass

        self.events = queue.Queue()
        self.worker = Worker(self.events, motor_factory)
        self.worker.start()

        # Datos en vivo (listas para poder buscar la ventana de tiempo con bisect)
        self.t, self.des, self.meas = [], [], []
        self.table = None   # (t, {columna: array}) del CSV cargado
        self.ref = None     # (t, angulos) de la ultima ejecucion pedida
        self.run_log = None
        self.connected = False
        self.running = False
        self._last_draw = 0.0
        self._draw_pending = False

        self._build_controls()
        self._build_plots()
        self._update_controls()
        self.refresh_ports()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(EVENT_MS, self._poll_events)

    # --- construccion de la ventana -----------------------------------------

    def _build_controls(self):
        side = ttk.Frame(self.root, padding=8)
        side.pack(side="left", fill="y")

        conn = ttk.LabelFrame(side, text="Conexion", padding=6)
        conn.pack(fill="x")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(conn, textvariable=self.port_var, width=12)
        self.port_box.grid(row=0, column=0, sticky="we")
        ttk.Button(conn, text="Actualizar", command=self.refresh_ports).grid(row=0, column=1, padx=4)
        ttk.Label(conn, text="ID del motor").grid(row=1, column=0, sticky="w", pady=(4, 0))
        self.id_var = tk.IntVar(value=1)
        ttk.Spinbox(conn, from_=1, to=32, textvariable=self.id_var, width=5).grid(
            row=1, column=1, pady=(4, 0))
        self.connect_btn = ttk.Button(conn, text="Conectar", command=self.toggle_connection)
        self.connect_btn.grid(row=2, column=0, columnspan=2, sticky="we", pady=(6, 0))

        pos = ttk.LabelFrame(side, text="Posicion manual", padding=6)
        pos.pack(fill="x", pady=8)
        self.angle_var = tk.StringVar(value="0")
        self.speed_var = tk.StringVar(value="60")
        ttk.Label(pos, text="Angulo [deg]").grid(row=0, column=0, sticky="w")
        angle_entry = ttk.Entry(pos, textvariable=self.angle_var, width=9)
        angle_entry.grid(row=0, column=1, padx=4)
        angle_entry.bind("<Return>", lambda e: self.send_position())
        ttk.Label(pos, text="Velocidad [dps]").grid(row=1, column=0, sticky="w")
        ttk.Entry(pos, textvariable=self.speed_var, width=9).grid(row=1, column=1, padx=4)
        self.send_btn = ttk.Button(pos, text="Enviar posicion", command=self.send_position)
        self.send_btn.grid(row=2, column=0, columnspan=2, sticky="we", pady=(6, 0))
        jog = ttk.Frame(pos)
        jog.grid(row=3, column=0, columnspan=2, pady=(4, 0))
        self.jog_btns = []
        for delta in (-10, -1, 1, 10):
            btn = ttk.Button(jog, text=f"{delta:+d}", width=5,
                             command=lambda d=delta: self.jog(d))
            btn.pack(side="left", padx=1)
            self.jog_btns.append(btn)
        actions = ttk.Frame(pos)
        actions.grid(row=4, column=0, columnspan=2, sticky="we", pady=(6, 0))
        actions.columnconfigure((0, 1), weight=1)
        self.zero_btn = ttk.Button(actions, text="Ir a 0", command=self.go_zero)
        self.zero_btn.grid(row=0, column=0, sticky="we")
        self.stop_btn = ttk.Button(actions, text="Parar motor", command=self.stop_motor)
        self.stop_btn.grid(row=0, column=1, sticky="we", padx=(4, 0))

        traj = ttk.LabelFrame(side, text="Trayectoria desde CSV", padding=6)
        traj.pack(fill="x")
        ttk.Button(traj, text="Cargar CSV...", command=self.load_csv).grid(
            row=0, column=0, columnspan=2, sticky="we")
        self.file_label = ttk.Label(traj, text="(ninguno)", foreground="gray")
        self.file_label.grid(row=1, column=0, columnspan=2, sticky="w")
        self.column_var = tk.StringVar()
        ttk.Label(traj, text="Columna").grid(row=2, column=0, sticky="w")
        self.column_box = ttk.Combobox(traj, textvariable=self.column_var, state="readonly", width=16)
        self.column_box.grid(row=2, column=1, padx=4)
        self.column_box.bind("<<ComboboxSelected>>", lambda e: self._update_info())
        self.scale_var = tk.StringVar(value="1")
        self.cycles_var = tk.StringVar(value="1")
        self.offset_var = tk.StringVar(value="0")
        self.vmax_var = tk.StringVar(value=str(MAX_RUN_SPEED_DPS))
        for row, (label, var) in enumerate([("Escala de tiempo", self.scale_var),
                                            ("Ciclos", self.cycles_var),
                                            ("Offset [deg]", self.offset_var),
                                            ("Vel. max [dps]", self.vmax_var)], start=3):
            ttk.Label(traj, text=label).grid(row=row, column=0, sticky="w")
            ttk.Entry(traj, textvariable=var, width=9).grid(row=row, column=1, padx=4)
            var.trace_add("write", lambda *a: self._update_info())
        self.info_label = ttk.Label(traj, text="", wraplength=250, justify="left")
        self.info_label.grid(row=7, column=0, columnspan=2, sticky="w", pady=(4, 0))
        self.run_btn = ttk.Button(traj, text="Ejecutar", command=self.execute)
        self.run_btn.grid(row=8, column=0, sticky="we", pady=(6, 0))
        self.abort_btn = ttk.Button(traj, text="Abortar", command=self.abort_run)
        self.abort_btn.grid(row=8, column=1, sticky="we", pady=(6, 0), padx=4)

    def _build_bottom(self, parent):
        """Franja bajo las graficas: opciones de la grafica, exportacion y mensajes."""
        bottom = ttk.Frame(parent)
        bottom.pack(side="bottom", fill="x", pady=(8, 0))

        data = ttk.LabelFrame(bottom, text="Grafica y datos", padding=6)
        data.pack(side="left", fill="y")
        self.window_var = tk.StringVar(value="20")
        ttk.Label(data, text="Ventana [s] (0 = todo)").grid(row=0, column=0, sticky="w")
        ttk.Entry(data, textvariable=self.window_var, width=6).grid(row=0, column=1, padx=4)
        self.pause_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(data, text="Pausar grafica (sigue grabando)",
                        variable=self.pause_var).grid(row=1, column=0, columnspan=2, sticky="w")
        ttk.Button(data, text="Limpiar datos en vivo", command=self.clear_live).grid(
            row=2, column=0, columnspan=2, sticky="we", pady=(4, 0))
        self.export_live_btn = ttk.Button(data, text="Exportar datos en vivo...",
                                          command=self.export_live)
        self.export_live_btn.grid(row=3, column=0, columnspan=2, sticky="we", pady=(4, 0))
        self.export_run_btn = ttk.Button(data, text="Exportar ultima ejecucion...",
                                         command=self.export_run)
        self.export_run_btn.grid(row=4, column=0, columnspan=2, sticky="we", pady=(4, 0))

        accel = ttk.LabelFrame(bottom, text="Aceleracion del motor [dps/s]", padding=6)
        accel.pack(side="left", fill="y", padx=(8, 0))
        ttk.Label(accel, text="Actual").grid(row=0, column=1)
        ttk.Label(accel, text="Nuevo").grid(row=0, column=2)
        self.accel_cur, self.accel_new = [], []
        for row, label in enumerate(["Pos. aceleracion", "Pos. frenado",
                                     "Vel. aceleracion", "Vel. frenado"], start=1):
            ttk.Label(accel, text=label).grid(row=row, column=0, sticky="w")
            cur = ttk.Label(accel, text="-", width=7, anchor="e")
            cur.grid(row=row, column=1)
            self.accel_cur.append(cur)
            if row <= 2:  # el 0xA4 solo usa la planificacion de posicion
                var = tk.StringVar()
                ttk.Entry(accel, textvariable=var, width=8).grid(row=row, column=2, padx=4)
                self.accel_new.append(var)
        accel_buttons = ttk.Frame(accel)
        accel_buttons.grid(row=5, column=0, columnspan=3, pady=(6, 0))
        self.accel_read_btn = ttk.Button(accel_buttons, text="Leer", width=7,
                                         command=self.read_accel)
        self.accel_read_btn.pack(side="left", padx=1)
        self.accel_max_btn = ttk.Button(accel_buttons, text=f"Poner {MAX_ACCEL_DPS2}",
                                        command=self.fill_max_accel)
        self.accel_max_btn.pack(side="left", padx=1)
        self.accel_write_btn = ttk.Button(accel_buttons, text="Escribir", width=8,
                                          command=self.write_accel)
        self.accel_write_btn.pack(side="left", padx=1)
        ttk.Label(accel, text=f"Rango {MIN_ACCEL_DPS2}-{MAX_ACCEL_DPS2}. Escribir guarda en la ROM.",
                  foreground="gray").grid(row=6, column=0, columnspan=3, sticky="w")

        mode = ttk.LabelFrame(bottom, text="Modo de control", padding=6)
        mode.pack(side="left", fill="y", padx=(8, 0))
        self.mode_var = tk.IntVar(value=MODE_POSITION)
        self.kp_var = tk.StringVar(value="20")
        self.kd_var = tk.StringVar(value="1")
        self.vff_var = tk.StringVar(value="1")
        ttk.Radiobutton(mode, text="0xA4 posicion (planificador)", value=MODE_POSITION,
                        variable=self.mode_var, command=self._update_mode_fields).grid(
            row=0, column=0, columnspan=2, sticky="w")
        ttk.Radiobutton(mode, text="0x400 movimiento (kp, kd)", value=MODE_MOTION,
                        variable=self.mode_var, command=self._update_mode_fields).grid(
            row=1, column=0, columnspan=2, sticky="w")
        self.mode_entries = []
        for row, (label, var) in enumerate([("kp [N.m/rad]", self.kp_var),
                                            ("kd [N.m/(rad/s)]", self.kd_var),
                                            ("Veloc. ref. (x)", self.vff_var)], start=2):
            ttk.Label(mode, text=label).grid(row=row, column=0, sticky="w")
            entry = ttk.Entry(mode, textvariable=var, width=8)
            entry.grid(row=row, column=1, padx=4)
            self.mode_entries.append(entry)
        self.mode_test_btn = ttk.Button(mode, text="Probar 0x400 (+5 deg)",
                                        command=self.motion_test)
        self.mode_test_btn.grid(row=5, column=0, columnspan=2, sticky="we", pady=(6, 0))
        ttk.Label(mode, text="El modo vale para Enviar posicion,\nIr a 0, jog y CSV (0x400 sin\n"
                             "limite de velocidad).", foreground="gray").grid(
            row=6, column=0, columnspan=2, sticky="w")
        self._update_mode_fields()

        torque = ttk.LabelFrame(bottom, text="Torque (0xA1)", padding=6)
        torque.pack(side="left", fill="y", padx=(8, 0))
        self.current_var = tk.StringVar(value="0.5")
        self.duration_var = tk.StringVar(value="1000")
        ttk.Label(torque, text="Corriente [A]").grid(row=0, column=0, sticky="w")
        ttk.Entry(torque, textvariable=self.current_var, width=8).grid(row=0, column=1, padx=4)
        ttk.Label(torque, text="Duracion [ms]").grid(row=1, column=0, sticky="w")
        ttk.Entry(torque, textvariable=self.duration_var, width=8).grid(row=1, column=1, padx=4)
        self.torque_btn = ttk.Button(torque, text="Aplicar torque", command=self.apply_torque)
        self.torque_btn.grid(row=2, column=0, columnspan=2, sticky="we", pady=(6, 0))
        ttk.Label(torque, text=f"Max +-{MAX_TORQUE_A:g} A, {MIN_TORQUE_MS}-{MAX_TORQUE_MS} ms.\n"
                               "Al acabar el tiempo el motor\nse para. Sin limite de velocidad.",
                  foreground="gray").grid(row=3, column=0, columnspan=2, sticky="w")

        messages = ttk.Frame(bottom)
        messages.pack(side="left", fill="both", expand=True, padx=(8, 0))
        self.readout = ttk.Label(messages, text="Medida: -   Deseada: -",
                                 font=("Segoe UI", 11, "bold"))
        self.readout.pack(anchor="w")
        self.log_text = tk.Text(messages, height=6, state="disabled", wrap="word")
        self.log_text.pack(fill="both", expand=True, pady=(4, 0))
        self.log_text.tag_configure("error", foreground="firebrick")

    def _build_plots(self):
        right = ttk.Frame(self.root)
        right.pack(side="right", fill="both", expand=True, padx=(0, 8), pady=8)
        self._build_bottom(right)
        self.notebook = ttk.Notebook(right)
        self.notebook.pack(side="top", fill="both", expand=True)

        live_tab = ttk.Frame(self.notebook)
        self.notebook.add(live_tab, text="En vivo")
        # Margenes fijos: el layout "constrained" recalcula todo en cada fotograma y
        # hace lenta la grafica en vivo
        self.live_fig = Figure()
        self.ax, self.ax_err = self.live_fig.subplots(2, 1, sharex=True,
                                                      gridspec_kw={"height_ratios": [3, 1]})
        self.live_fig.subplots_adjust(left=0.09, right=0.985, top=0.97, bottom=0.11, hspace=0.08)
        (self.l_des,) = self.ax.plot([], [], color="tab:orange", drawstyle="steps-post",
                                     label="Deseada")
        (self.l_meas,) = self.ax.plot([], [], color="tab:blue", label="Medida (encoder)")
        (self.l_err,) = self.ax_err.plot([], [], color="tab:red")
        self.ax.set_ylabel("Angulo [deg]")
        self.ax.legend(loc="upper left")
        self.ax_err.set_ylabel("Error [deg]")
        self.ax_err.set_xlabel("Tiempo [s]")
        for axis in (self.ax, self.ax_err):
            axis.grid(True)
        self.live_canvas = FigureCanvasTkAgg(self.live_fig, master=live_tab)
        self.live_canvas.mpl_connect("draw_event", self._on_live_draw)
        NavigationToolbar2Tk(self.live_canvas, live_tab).pack(side="bottom", fill="x")
        self.live_canvas.get_tk_widget().pack(fill="both", expand=True)

        run_tab = ttk.Frame(self.notebook)
        self.notebook.add(run_tab, text="Ultima ejecucion")
        self.run_fig = Figure(layout="constrained")
        self.run_ax, self.run_ax_err = self.run_fig.subplots(2, 1, sharex=True,
                                                             gridspec_kw={"height_ratios": [3, 1]})
        self.run_canvas = FigureCanvasTkAgg(self.run_fig, master=run_tab)
        NavigationToolbar2Tk(self.run_canvas, run_tab).pack(side="bottom", fill="x")
        self.run_canvas.get_tk_widget().pack(fill="both", expand=True)
        self._draw_run_placeholder()
        self.notebook.bind("<<NotebookTabChanged>>", lambda e: self._redraw_live(force=True))

    def _on_live_draw(self, event):
        self._draw_pending = False

    def _request_live_draw(self):
        self._draw_pending = True
        self.live_canvas.draw_idle()

    def _draw_run_placeholder(self):
        for axis in (self.run_ax, self.run_ax_err):
            axis.clear()
            axis.grid(True)
        self.run_ax.set_title("Carga un CSV y pulsa Ejecutar")
        self.run_canvas.draw_idle()

    # --- utilidades ---------------------------------------------------------

    def log(self, message, error=False):
        stamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{stamp} {message}\n", "error" if error else "")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _float(self, var, name):
        try:
            return float(var.get().replace(",", "."))
        except ValueError:
            raise ValueError(f"{name}: valor no numerico") from None

    def _update_controls(self):
        idle = self.connected and not self.running
        for btn in [self.send_btn, self.zero_btn, self.stop_btn, *self.jog_btns,
                    self.accel_read_btn, self.accel_max_btn, self.accel_write_btn,
                    self.mode_test_btn, self.torque_btn]:
            btn.configure(state="normal" if idle else "disabled")
        self.run_btn.configure(state="normal" if idle and self.table else "disabled")
        self.abort_btn.configure(state="normal" if self.running else "disabled")
        self.connect_btn.configure(text="Desconectar" if self.connected else "Conectar",
                                   state="disabled" if self.running else "normal")
        self.export_run_btn.configure(state="normal" if self.run_log else "disabled")

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_box.configure(values=ports)
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    # --- acciones de los botones ----------------------------------------------

    def toggle_connection(self):
        if self.connected:
            self.worker.cmds.put(("disconnect", ()))
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Conexion", "Elige el puerto de la ESP32")
            return
        self.connect_btn.configure(state="disabled")
        self.log(f"Conectando a {port} (espera unos 3 s)...")
        self.worker.cmds.put(("connect", (port, self.id_var.get())))

    def send_position(self, angle=None):
        try:
            if angle is None:
                angle = self._float(self.angle_var, "Angulo")
            speed = self._float(self.speed_var, "Velocidad")
            if abs(angle) > MAX_ANGLE_DEG:
                raise ValueError(f"el angulo debe estar entre +-{MAX_ANGLE_DEG:g} grados")
            if not 1 <= speed <= MAX_SPEED_DPS:
                raise ValueError(f"la velocidad debe estar entre 1 y {MAX_SPEED_DPS} dps")
        except ValueError as exc:
            messagebox.showerror("Posicion", str(exc))
            return
        try:
            mode_cfg = self._mode_config()
        except ValueError as exc:
            messagebox.showerror("Posicion", str(exc))
            return
        if mode_cfg[0] == MODE_MOTION:
            here = self.meas[-1] if self.meas else None
            if here is None or abs(angle - here) > 15.0:
                if not messagebox.askokcancel(
                        "Posicion en modo 0x400",
                        f"En modo 0x400 no hay planificador ni limite de velocidad: el motor "
                        f"aplica kp*error de golpe"
                        + (f" ({angle - here:+.1f} grados)." if here is not None else ".")
                        + "\nContinuar?"):
                    return
        self.angle_var.set(f"{angle:g}")
        self.worker.cmds.put(("position", (angle, int(speed), mode_cfg)))

    def apply_torque(self):
        try:
            current = self._float(self.current_var, "Corriente")
            duration = self._float(self.duration_var, "Duracion")
            if abs(current) > MAX_TORQUE_A:
                raise ValueError(f"la corriente debe estar entre +-{MAX_TORQUE_A:g} A")
            if not MIN_TORQUE_MS <= duration <= MAX_TORQUE_MS:
                raise ValueError(f"la duracion debe estar entre {MIN_TORQUE_MS} y {MAX_TORQUE_MS} ms")
        except ValueError as exc:
            messagebox.showerror("Torque", str(exc))
            return
        if not messagebox.askokcancel(
                "Aplicar torque",
                f"Se aplicaran {current:g} A durante {duration:.0f} ms sin limite de velocidad "
                "y luego el motor se parara.\nAsegurate de que el eje esta libre y lejos de "
                "topes. \"Parar motor\" lo detiene antes. Continuar?"):
            return
        self.log(f"Torque {current:g} A durante {duration:.0f} ms")
        self.worker.cmds.put(("torque", (current, int(duration))))

    def go_zero(self):
        self.send_position(0.0)

    def jog(self, delta):
        base = self.worker.setpoint
        if math.isnan(base):
            base = self.meas[-1] if self.meas else 0.0
        self.send_position(round(base + delta, 2))

    def stop_motor(self):
        self.worker.cmds.put(("stop", ()))

    def clear_live(self):
        self.t.clear()
        self.des.clear()
        self.meas.clear()
        self.worker.t0 = time.monotonic()
        self._redraw_live(force=True)

    def _update_mode_fields(self):
        state = "normal" if self.mode_var.get() == MODE_MOTION else "disabled"
        for entry in self.mode_entries:
            entry.configure(state=state)

    def _mode_config(self):
        """(modo, kp, kd, vff) de los campos; lanza ValueError si algo no vale."""
        mode = self.mode_var.get()
        kp = self._float(self.kp_var, "kp")
        kd = self._float(self.kd_var, "kd")
        vff = self._float(self.vff_var, "Veloc. ref.")
        if mode == MODE_MOTION:
            if not 0 <= kp <= MM_KP_MAX:
                raise ValueError(f"kp debe estar entre 0 y {MM_KP_MAX:g}")
            if not 0 <= kd <= MM_KD_MAX:
                raise ValueError(f"kd debe estar entre 0 y {MM_KD_MAX:g}")
            if not 0 <= vff <= MM_VFF_MAX:
                raise ValueError(f"la velocidad de referencia debe estar entre 0 y {MM_VFF_MAX:g}")
        else:  # en modo 0xA4 los campos no se usan: valores validos para la ESP32
            kp, kd, vff = 20.0, 1.0, 1.0
        return mode, kp, kd, vff

    def motion_test(self):
        if not messagebox.askokcancel(
                "Probar modo 0x400",
                "El motor se movera unos +5 grados con ganancias bajas (kp 8, kd 0.5) y "
                "volvera. Sirve para comprobar que el modo 0x400 usa el mismo cero y sentido "
                "que el encoder.\nAsegurate de que el eje esta libre. Continuar?"):
            return
        self.worker.cmds.put(("motion_test", ()))

    def read_accel(self):
        self.worker.cmds.put(("read_accel", ()))

    def fill_max_accel(self):
        for var in self.accel_new:
            var.set(str(MAX_ACCEL_DPS2))

    def write_accel(self):
        values = {}
        for index, var in enumerate(self.accel_new):
            text = var.get().strip()
            if not text:
                continue
            try:
                value = int(text)
            except ValueError:
                messagebox.showerror("Aceleracion", f"{ACCEL_NAMES[index]}: escribe un entero en dps/s")
                return
            if not MIN_ACCEL_DPS2 <= value <= MAX_ACCEL_DPS2:
                messagebox.showerror("Aceleracion",
                                     f"{ACCEL_NAMES[index]}: debe estar entre "
                                     f"{MIN_ACCEL_DPS2} y {MAX_ACCEL_DPS2} dps/s")
                return
            values[index] = value
        if not values:
            messagebox.showinfo("Aceleracion", "Escribe al menos un valor nuevo")
            return
        if not messagebox.askokcancel(
                "Escribir aceleracion",
                "Esto se guarda en la memoria permanente (ROM) del motor.\n"
                "El motor debe estar parado. Continuar?"):
            return
        self.worker.cmds.put(("write_accel", (values,)))

    def load_csv(self):
        path = filedialog.askopenfilename(title="Cargar CSV de angulos",
                                          filetypes=[("CSV", "*.csv"), ("Todos", "*.*")])
        if not path:
            return
        try:
            t, columns = tracking.read_table(path)
        except (OSError, ValueError) as exc:
            messagebox.showerror("Cargar CSV", str(exc))
            return
        self.table = (t, columns)
        self.file_label.configure(text=Path(path).name, foreground="black")
        self.column_box.configure(values=list(columns))
        self.column_var.set(next(iter(columns)))
        self.log(f"CSV cargado: {Path(path).name} ({len(t)} filas, {len(columns)} columnas)")
        self._update_info()
        self._update_controls()

    def _build_reference(self):
        t, columns = self.table
        return tracking.build_reference(
            t, columns[self.column_var.get()],
            time_scale=self._float(self.scale_var, "Escala de tiempo"),
            cycles=int(self._float(self.cycles_var, "Ciclos")),
            offset=self._float(self.offset_var, "Offset"))

    def _update_info(self):
        if not self.table or not self.column_var.get():
            self.info_label.configure(text="")
            return
        try:
            ref_t, ref_a = self._build_reference()
            vmax = self._float(self.vmax_var, "Vel. max")
        except (ValueError, KeyError) as exc:
            self.info_label.configure(text=str(exc), foreground="firebrick")
            return
        peak = peak_speed_dps(ref_t.tolist(), ref_a.tolist())
        text = (f"{len(ref_t)} puntos, {ref_t[-1]:.2f} s, pico {peak:.0f} dps, "
                f"rango {ref_a.min():.1f} a {ref_a.max():.1f} deg")
        color = "black"
        if len(ref_t) > MAX_POINTS:
            text += f"\nDemasiados puntos (maximo {MAX_POINTS}): baja los ciclos."
            color = "firebrick"
        elif peak > min(vmax, MAX_RUN_SPEED_DPS):
            suggested = float(self.scale_var.get().replace(",", ".")) * peak / min(vmax, MAX_RUN_SPEED_DPS) * 1.1
            text += f"\nPide mas velocidad que el limite: prueba escala de tiempo {suggested:.1f}."
            color = "darkorange"
        self.info_label.configure(text=text, foreground=color)

    def execute(self):
        try:
            ref_t, ref_a = self._build_reference()
            vmax = self._float(self.vmax_var, "Vel. max")
            start_speed = self._float(self.speed_var, "Velocidad")
            if len(ref_t) > MAX_POINTS:
                raise ValueError(f"demasiados puntos ({len(ref_t)}); el maximo es {MAX_POINTS}")
            if np.max(np.abs(ref_a)) > MAX_ANGLE_DEG:
                raise ValueError(f"hay angulos fuera de +-{MAX_ANGLE_DEG:g} grados")
            if not 1 <= vmax <= MAX_RUN_SPEED_DPS:
                raise ValueError(f"vel. max debe estar entre 1 y {MAX_RUN_SPEED_DPS} dps")
            if not 1 <= start_speed <= MAX_SPEED_DPS:
                raise ValueError(f"la velocidad inicial debe estar entre 1 y {MAX_SPEED_DPS} dps")
            mode_cfg = self._mode_config()
        except (ValueError, KeyError) as exc:
            messagebox.showerror("Ejecutar", str(exc))
            return
        if mode_cfg[0] == MODE_MOTION:
            mode_text = (f"Modo 0x400: kp {mode_cfg[1]:g}, kd {mode_cfg[2]:g}, velocidad de "
                         f"referencia x{mode_cfg[3]:g}. Es un lazo de par: si nunca lo has "
                         f"probado, usa antes \"Probar 0x400\".\n\n")
        else:
            mode_text = "Modo 0xA4 (posicion).\n\n"
        if not messagebox.askokcancel(
                "Ejecutar trayectoria",
                f"{mode_text}El motor ira primero a {ref_a[0]:.1f} grados y luego seguira la "
                f"trayectoria ({ref_t[-1]:.2f} s).\nAsegurate de que el eje esta libre."):
            return
        self.ref = (ref_t, ref_a)
        self.worker.cmds.put(("play", (ref_t.tolist(), ref_a.tolist(), int(vmax),
                                       int(start_speed), mode_cfg)))

    def abort_run(self):
        self.log("Abortando...", error=True)
        self.worker.abort()

    def export_live(self):
        if not self.t:
            messagebox.showinfo("Exportar", "Todavia no hay datos en vivo")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile=time.strftime("vivo_%Y%m%d_%H%M%S.csv"))
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["t_s", "deseada_deg", "medida_deg", "error_deg"])
            for t, d, m in zip(self.t, self.des, self.meas):
                writer.writerow([f"{t:.4f}", "" if math.isnan(d) else d, m,
                                 "" if math.isnan(d) else round(m - d, 4)])
        self.log(f"Exportado: {path}")

    def export_run(self):
        if not self.run_log:
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile=time.strftime("ejecucion_%Y%m%d_%H%M%S.csv"))
        if not path:
            return
        tracking.save_log(self.run_log, *self.ref, path)
        self.log(f"Exportado: {path}")

    # --- eventos del hilo de comunicacion -------------------------------------------

    def _poll_events(self):
        changed = False
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            try:
                changed |= self._handle_event(event)
            except Exception as exc:  # un fallo al dibujar no debe parar la ventana
                self.log(f"error interno: {exc!r}", error=True)
        if changed:
            self._redraw_live()
        self.root.after(EVENT_MS, self._poll_events)

    def _handle_event(self, event):
        kind = event[0]
        if kind == "sample":
            _, t, desired, measured = event
            self.t.append(t)
            self.des.append(desired)
            self.meas.append(measured)
            return True
        if kind == "connected":
            self.connected = True
            self.log(f"Conectado a {event[1]}, motor {event[2]}")
            self.read_accel()
        elif kind == "disconnected":
            self.connected = False
            self.running = False
            self.log(f"Desconectado: {event[1]}")
        elif kind == "error":
            self.log(event[1], error=True)
            self.connect_btn.configure(state="normal")
        elif kind == "status":
            self.log(event[1])
        elif kind == "accel":
            values = [event[1][name] for name in ACCEL_NAMES]
            for label, value in zip(self.accel_cur, values):
                label.configure(text=str(value))
            for var, value in zip(self.accel_new, values):
                if not var.get().strip():
                    var.set(str(value))
            self.log("Aceleraciones (pos. acel, pos. freno, vel. acel, vel. freno): "
                     + ", ".join(str(v) for v in values))
        elif kind == "motion_test":
            _, start, end = event
            delta = end - start
            verdict = ("OK: mismo cero y sentido que el encoder" if 3.0 <= delta <= 7.0
                       else "NO coincide con lo esperado (+5): no uses el modo 0x400 hasta aclararlo")
            self.log(f"Prueba 0x400: de {start:.2f} a {end:.2f} deg (cambio {delta:+.2f}). {verdict}",
                     error=not 3.0 <= delta <= 7.0)
        elif kind == "run_started":
            self.running = True
        elif kind == "run_failed":
            self.running = False
            self.log(f"Trayectoria fallida: {event[1]}", error=True)
        elif kind == "run_done":
            self.running = False
            self._show_run(event[1])
        self._update_controls()
        return False

    def _show_run(self, log):
        self.run_log = log
        ref_t, ref_a = self.ref
        try:
            result = tracking.analyse(log, ref_t, ref_a)
        except ValueError as exc:
            self.log(f"Ejecucion sin datos utiles: {exc}", error=True)
            return
        for axis in (self.run_ax, self.run_ax_err):
            axis.clear()
            axis.grid(True)
        self.run_ax.plot(ref_t, ref_a, color="tab:orange", label="Deseada (CSV)")
        self.run_ax.plot(result["t"], result["meas"], color="tab:blue", label="Medida (encoder)")
        self.run_ax.set_ylabel("Angulo [deg]")
        self.run_ax.legend(loc="upper left")
        self.run_ax.set_title(
            f"RMS {result['rms']:.2f} deg, maximo {result['max_abs']:.2f} deg, "
            f"medio {result['mean']:.2f} deg, retardo ~{result['lag_s'] * 1000:.0f} ms",
            fontsize=10)
        self.run_ax_err.plot(result["t"], result["error"], color="tab:red")
        self.run_ax_err.set_ylabel("Medida - deseada [deg]")
        self.run_ax_err.set_xlabel("Tiempo [s]")
        self.run_canvas.draw_idle()
        self.notebook.select(1)
        self.log(f"Ejecucion terminada: RMS {result['rms']:.2f} deg, maximo {result['max_abs']:.2f} deg, "
                 f"{log.missed} pasos sin lectura")

    def _redraw_live(self, force=False):
        now = time.monotonic()
        if not force:
            # Sin redibujar si la grafica esta en pausa, si el dibujo anterior sigue
            # pendiente o si se esta viendo la otra pestana (los datos se siguen guardando)
            waiting = self._draw_pending and now - self._last_draw < 1.0
            if (self.pause_var.get() or waiting or now - self._last_draw < REDRAW_S
                    or self.notebook.index(self.notebook.select()) != 0):
                return
        self._last_draw = now
        if not self.t:
            for line in (self.l_des, self.l_meas, self.l_err):
                line.set_data([], [])
            self._request_live_draw()
            return
        try:
            window = float(self.window_var.get().replace(",", "."))
        except ValueError:
            window = 20.0
        tmax = self.t[-1]
        start = 0 if window <= 0 else bisect_left(self.t, tmax - window)
        t = np.array(self.t[start:])
        des = np.array(self.des[start:])
        meas = np.array(self.meas[start:])
        err = meas - des
        self.l_des.set_data(t, des)
        self.l_meas.set_data(t, meas)
        self.l_err.set_data(t, err)
        if window > 0:
            self.ax.set_xlim(max(0.0, tmax - window), max(tmax, window))
        else:
            self.ax.set_xlim(t[0], max(tmax, t[0] + 1))
        values = np.concatenate([meas, des[np.isfinite(des)]])
        margin = max(2.0, 0.1 * (values.max() - values.min()))
        self.ax.set_ylim(values.min() - margin, values.max() + margin)
        finite_err = np.abs(err[np.isfinite(err)])
        limit = max(1.0, 1.2 * finite_err.max()) if len(finite_err) else 1.0
        self.ax_err.set_ylim(-limit, limit)
        self._request_live_draw()
        desired = "-" if math.isnan(self.des[-1]) else f"{self.des[-1]:.2f} deg"
        self.readout.configure(text=f"Medida: {self.meas[-1]:.2f} deg   Deseada: {desired}")

    def on_close(self):
        if self.running:
            self.worker.abort()
        self.worker.shutdown()
        self.root.destroy()


def main():
    try:  # nitidez en pantallas con escalado de Windows
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (ImportError, AttributeError, OSError):
        pass
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
