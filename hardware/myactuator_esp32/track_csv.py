"""Envia una columna de leg_motor_angles_one_cycle.csv al motor y compara el angulo
enviado con el que mide el encoder del MyActuator.

Ejemplos:
    python track_csv.py --port COM3 --column fr_hip_motor
    python track_csv.py --port COM3 --column fr_knee_motor --time-scale 2 --cycles 3
    python track_csv.py --port COM3 --column br_ankle_motor --offset -10 --no-show

El angulo del CSV se envia tal cual (mas --offset). Si el cero del motor no coincide
con el del modelo, usa --offset o mira el error medio del resumen.
"""
import argparse
import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt

from myactuator_esp32 import MAX_RUN_SPEED_DPS, MyActuator, peak_speed_dps
from tracking import analyse, build_reference, read_table, save_log

DEFAULT_CSV = (Path(__file__).resolve().parent.parent / "leg_transmission"
               / "leg_motor_angles_one_cycle.csv")


def load_column(path, column):
    t, columns = read_table(path)
    if column not in columns:
        sys.exit(f"Columna {column!r} no existe. Opciones: {', '.join(columns)}")
    return t, columns[column]


def plot(result, ref_t, ref_a, column, path, show):
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
    ax1.plot(ref_t, ref_a, label="enviado (CSV)")
    ax1.plot(result["t"], result["meas"], label="medido (encoder)")
    ax1.set_ylabel("angulo [deg]")
    ax1.set_title(f"{column}: RMS {result['rms']:.2f} deg, maximo {result['max_abs']:.2f} deg, "
                  f"retardo ~{result['lag_s'] * 1000:.0f} ms")
    ax1.legend()
    ax1.grid(True)
    ax2.plot(result["t"], result["error"], color="tab:red")
    ax2.set_ylabel("medido - enviado [deg]")
    ax2.set_xlabel("tiempo [s]")
    ax2.grid(True)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    if show:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="puerto de la ESP32, por ejemplo COM3")
    parser.add_argument("--column", required=True, help="columna del CSV, por ejemplo fr_hip_motor")
    parser.add_argument("--csv", default=str(DEFAULT_CSV))
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--time-scale", type=float, default=1.0,
                        help="multiplica los tiempos (2 = el doble de lento)")
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--offset", type=float, default=0.0, help="grados que se suman al CSV")
    parser.add_argument("--max-speed", type=int, default=MAX_RUN_SPEED_DPS,
                        help="limite de velocidad en dps")
    parser.add_argument("--out", help="CSV de salida (por defecto track_<columna>_<fecha>.csv)")
    parser.add_argument("--no-show", action="store_true", help="solo guardar la grafica")
    args = parser.parse_args()

    t, angles = load_column(args.csv, args.column)
    ref_t, ref_a = build_reference(t, angles, args.time_scale, args.cycles, args.offset)

    peak = peak_speed_dps(list(ref_t), list(ref_a))
    print(f"{len(ref_t)} puntos, {ref_t[-1]:.2f} s, velocidad pico {peak:.0f} dps, "
          f"rango {ref_a.min():.1f} a {ref_a.max():.1f} deg")
    if peak > args.max_speed:
        suggested = args.time_scale * peak / args.max_speed * 1.1
        print(f"AVISO: la trayectoria pide {peak:.0f} dps y el limite es {args.max_speed} dps; "
              f"el motor no podra seguirla. Prueba --time-scale {suggested:.1f}")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_csv = Path(args.out) if args.out else Path(f"track_{args.column}_{stamp}.csv")
    out_png = out_csv.with_suffix(".png")

    with MyActuator(args.port, motor_id=args.motor_id) as motor:
        print(f"Angulo inicial: {motor.read_angle():.2f} deg. Yendo al primer punto...")
        log = motor.play(list(ref_t), list(ref_a), max_speed_dps=args.max_speed)

    print(f"Ejecucion terminada: {len(log.t_cmd_s)} pasos, {log.missed} sin lectura")
    result = analyse(log, ref_t, ref_a)
    print(f"Error medido - enviado: RMS {result['rms']:.3f} deg, maximo {result['max_abs']:.3f} deg, "
          f"medio {result['mean']:.3f} deg")
    print(f"Retardo estimado: {result['lag_s'] * 1000:.0f} ms "
          f"(RMS con ese retardo: {result['rms_at_lag']:.3f} deg)")

    save_log(log, ref_t, ref_a, out_csv)
    plot(result, ref_t, ref_a, args.column, out_png, show=not args.no_show)
    print(f"Guardado: {out_csv} y {out_png}")


if __name__ == "__main__":
    main()
