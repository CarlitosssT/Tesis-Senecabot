"""Lectura de tablas de angulos y comparacion entre angulo enviado y medido.

Compartido por track_csv.py (linea de comandos) y gui.py (interfaz grafica).
"""
import csv

import numpy as np


def read_table(path):
    """CSV con el tiempo [s] en la primera columna y un angulo [deg] por columna.

    Acepta separador coma o punto y coma, con o sin fila de encabezado.
    Devuelve (t, {nombre_de_columna: array}).
    """
    with open(path, newline="", encoding="utf-8-sig") as f:
        text = f.read()
    first_line = text.splitlines()[0] if text.strip() else ""
    delimiter = ";" if ";" in first_line and "," not in first_line else ","
    rows = [r for r in csv.reader(text.splitlines(), delimiter=delimiter)
            if any(c.strip() for c in r)]
    if delimiter == ";":  # Excel en espanol: coma decimal
        rows = [[c.replace(",", ".") for c in r] for r in rows]
    if not rows:
        raise ValueError("el archivo esta vacio")
    try:
        [float(c) for c in rows[0]]
        header = ["time_s"] + [f"col{i}" for i in range(1, len(rows[0]))]
    except ValueError:
        header = [c.strip() for c in rows[0]]
        rows = rows[1:]
    if len(header) < 2:
        raise ValueError("se necesitan al menos dos columnas: tiempo y angulo")
    if len(rows) < 2:
        raise ValueError("se necesitan al menos 2 filas de datos")
    try:
        data = np.array([[float(c) for c in r[:len(header)]] for r in rows])
    except ValueError as exc:
        raise ValueError("hay valores no numericos o filas incompletas") from exc
    t = data[:, 0]
    if np.any(np.diff(t) <= 0):
        raise ValueError("el tiempo (primera columna) debe ser estrictamente creciente")
    return t, {name: data[:, i] for i, name in enumerate(header) if i > 0}


def build_reference(t, angles, time_scale=1.0, cycles=1, offset=0.0):
    """Empieza en t = 0, escala el tiempo, repite el ciclo y suma el offset de angulo."""
    if time_scale <= 0 or cycles < 1:
        raise ValueError("la escala de tiempo debe ser positiva y los ciclos al menos 1")
    t = (np.asarray(t) - t[0]) * time_scale
    period = t[-1] + np.median(np.diff(t))
    all_t = np.concatenate([t + k * period for k in range(cycles)])
    all_a = np.tile(np.asarray(angles), cycles) + offset
    return all_t, all_a


def analyse(log, ref_t, ref_a):
    """Error medido - referencia en los instantes de lectura, y retardo estimado."""
    t = np.array(log.t_meas_s)
    meas = np.array(log.meas_deg)
    valid = ~np.isnan(meas) & (t > 0)
    t, meas = t[valid], meas[valid]
    if len(t) == 0:
        raise ValueError("no hay ninguna lectura valida del encoder en el registro")
    error = meas - np.interp(t, ref_t, ref_a)
    rms = float(np.sqrt(np.mean(error ** 2)))

    # Retardo: cuanto hay que atrasar la referencia para que el error sea minimo
    lags = np.arange(0.0, 0.2, 0.001)
    rms_by_lag = [np.sqrt(np.mean((meas - np.interp(t - lag, ref_t, ref_a)) ** 2))
                  for lag in lags]
    best = int(np.argmin(rms_by_lag))
    return {"t": t, "meas": meas, "error": error, "rms": rms,
            "max_abs": float(np.max(np.abs(error))), "mean": float(np.mean(error)),
            "lag_s": float(lags[best]), "rms_at_lag": float(rms_by_lag[best])}


def save_log(log, ref_t, ref_a, path):
    """Guarda el registro de una ejecucion junto con la referencia y el error."""
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["t_cmd_s", "cmd_deg", "t_meas_s", "meas_deg", "ref_deg", "error_deg"])
        for t_cmd, cmd, t_meas, meas in zip(log.t_cmd_s, log.cmd_deg, log.t_meas_s,
                                            log.meas_deg):
            ref = float(np.interp(t_meas, ref_t, ref_a))
            writer.writerow([t_cmd, cmd, t_meas, meas, ref, meas - ref])
