"""Genera los CSV de excitacion para identificar el sistema (comando -> posicion medida).

    python make_sysid_signals.py            # escribe los CSV junto a este script
    python make_sysid_signals.py --plot     # ademas guarda sysid_preview.png

Cada archivo tiene dos columnas, time_s y angle_deg, empieza y termina en 0 grados y
cabe en la tabla de la ESP32 (1000 puntos). Se cargan tal cual en gui.py con escala de
tiempo 1, ciclos 1, offset 0 y velocidad maxima 600 dps.

  sysid_chirp.csv      barrido de frecuencia logaritmico 0.2-15 Hz: respuesta en frecuencia
  sysid_multisine.csv  suma de 24 senoidales (0.2-15 Hz) con fases de Schroeder, 2 periodos
                       de 5 s: bajo factor de cresta; los dos periodos permiten promediar
                       y estimar el ruido
  sysid_steps.csv      escalones de +-1, 2, 3 y 5 grados: tiempo de subida, sobreimpulso,
                       retardo y no linealidades (friccion, holgura)
  sysid_ramp.csv       triangular lenta (2 dps): histeresis por friccion y holgura

El primer y ultimo 0.3 s de chirp y multisine llevan una ventana suave: para identificar,
descarta esos tramos.
"""
import argparse
from pathlib import Path

import numpy as np

DT = 0.01            # s entre puntos (100 Hz)
N_POINTS = 1000      # tamano maximo de la tabla de la ESP32
FADE_S = 0.3         # ventana suave al principio y al final
MAX_SPEED_DPS = 250  # velocidad pico objetivo de las senales suaves (limite de la ESP32: 600)

T = np.arange(N_POINTS) * DT
DURATION = N_POINTS * DT


def fade_window():
    """0 -> 1 -> 0 con coseno elevado en los extremos."""
    ramp = 0.5 - 0.5 * np.cos(np.pi * np.clip(T / FADE_S, 0, 1))
    tail = 0.5 - 0.5 * np.cos(np.pi * np.clip((T[-1] - T) / FADE_S, 0, 1))
    return ramp * tail


def chirp(f0=0.2, f1=15.0, max_amp_deg=15.0):
    """Barrido logaritmico. La amplitud baja con la frecuencia para no pasar de
    MAX_SPEED_DPS: A(f) = min(max_amp, v / (2 pi f))."""
    k = f1 / f0
    phase = 2 * np.pi * f0 * DURATION / np.log(k) * (k ** (T / DURATION) - 1)
    freq = f0 * k ** (T / DURATION)
    amp = np.minimum(max_amp_deg, MAX_SPEED_DPS / (2 * np.pi * freq))
    return amp * np.sin(phase) * fade_window()


def multisine(period_s=5.0, f_max=15.0, n_components=24, max_pos_deg=12.0, rolloff=0.5):
    """Senoidales con fases de Schroeder, repetidas durante 2 periodos. La amplitud de
    cada una cae como f^-rolloff: con 1 la velocidad es plana, pero a 15 Hz quedaria por
    debajo de la holgura del reductor (0.25 deg); 0.5 es un compromiso."""
    df = 1.0 / period_s
    k_max = int(f_max / df)
    harmonics = np.unique(np.round(np.logspace(0, np.log10(k_max), n_components)).astype(int))
    freqs = harmonics * df
    n = len(freqs)
    phases = -np.pi * np.arange(n) * (np.arange(n) + 1) / n  # Schroeder
    amps = (2 * np.pi * freqs) ** -rolloff
    x = sum(a * np.sin(2 * np.pi * f * T + p) for a, f, p in zip(amps, freqs, phases))
    x = x * fade_window()
    speed = np.max(np.abs(np.diff(x))) / DT
    scale = min(max_pos_deg / np.max(np.abs(x)), MAX_SPEED_DPS / speed)
    return x * scale


def steps(levels_deg=(0, 1, 0, -1, 0, 2, 0, -2, 0, 3, 0, -3, 0, 5, 0, -5, 0, 0, 0, 0)):
    """Escalones de 0.5 s. El mayor (5 deg) pide 500 dps entre dos puntos."""
    per_level = N_POINTS // len(levels_deg)
    x = np.repeat(np.array(levels_deg, dtype=float), per_level)
    return np.concatenate([x, np.zeros(N_POINTS - len(x))])


def ramp(speed_dps=2.0, amp_deg=2.0):
    """Triangular lenta: 0 -> +amp -> -amp -> 0 ..., vuelve a 0 a los 10 s."""
    period = 4 * amp_deg / speed_dps
    phase = (T / period) % 1.0
    return amp_deg * np.where(phase < 0.25, 4 * phase,
                              np.where(phase < 0.75, 2 - 4 * phase, 4 * phase - 4))


SIGNALS = {"sysid_chirp": chirp, "sysid_multisine": multisine,
           "sysid_steps": steps, "sysid_ramp": ramp}


def write_csv(path, angles):
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write("time_s,angle_deg\n")
        for t, a in zip(T, angles):
            f.write(f"{t:.4f},{a:.4f}\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--plot", action="store_true", help="guarda sysid_preview.png")
    args = parser.parse_args()
    out = Path(__file__).resolve().parent

    signals = {name: fn() for name, fn in SIGNALS.items()}
    for name, x in signals.items():
        write_csv(out / f"{name}.csv", x)
        speed = np.max(np.abs(np.diff(x))) / DT
        print(f"{name}.csv: {len(x)} puntos, {DURATION:.1f} s, rango {x.min():.1f} a {x.max():.1f} deg, "
              f"velocidad pico {speed:.0f} dps, inicio {x[0]:.2f}, final {x[-1]:.2f}")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(len(signals), 1, figsize=(10, 9), sharex=True)
        for ax, (name, x) in zip(axes, signals.items()):
            ax.plot(T, x, drawstyle="steps-post" if name == "sysid_steps" else "default")
            ax.set_ylabel("[deg]")
            ax.set_title(name, loc="left", fontsize=9)
            ax.grid(True)
        axes[-1].set_xlabel("tiempo [s]")
        fig.tight_layout()
        fig.savefig(out / "sysid_preview.png", dpi=110)
        print("sysid_preview.png guardado")


if __name__ == "__main__":
    main()
