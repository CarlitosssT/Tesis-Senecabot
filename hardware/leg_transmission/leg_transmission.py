"""Transmision por paralelogramos: rodilla y tobillo movidos desde el torso.

La cadera sigue en accionamiento directo. La rodilla se mueve con un
paralelogramo como el del mecanismo de referencia: manivela en el eje de la
cadera -> biela paralela al muslo -> palanca fija a la tibia. El tobillo usa dos
en serie: manivela en el eje de la cadera -> biela paralela al muslo ->
balancin libre en el eje de la rodilla -> biela paralela a la tibia -> palanca
fija al metatarso. Si el motor no es coaxial con la cadera (motores en fila
dentro del torso), otro paralelogramo horizontal lleva su giro a la manivela.

Un paralelogramo copia orientaciones, asi que cada motor del torso fija la
orientacion ABSOLUTA (respecto al torso) de su segmento:

    m_hip   = hip                   (muslo)
    m_knee  = hip - knee            (tibia)
    m_ankle = hip - knee + ankle    (metatarso)

Convencion: la de senecabot_loco.xml, que es la de las columnas *_joint del
CSV (qpos de MuJoCo en grados). Cabeceo alrededor de +y del torso, 0 =
segmento colgando vertical, positivo = el segmento rota hacia atras. La
rodilla tiene eje -y en el modelo, por eso entra con signo menos.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Solo se lee: el ciclo de marcha vive en la carpeta servos/.
CSV_PATH = Path(__file__).resolve().parent.parent / "servos" / "leg_actuator_angles_one_cycle.csv"
OUT_CSV = Path(__file__).with_name("leg_motor_angles_one_cycle.csv")
OUT_PNG = Path(__file__).with_name("leg_transmission.png")
LEGS = ["fr", "fl", "br", "bl"]
JOINTS = ["hip", "knee", "ankle"]
STAGES = ["knee", "ankle_upper", "ankle_lower"]

# Muslo, tibia, metatarso [m] (iguales a senecabot_loco.xml).
SEGMENT_LENGTHS = {"front": (0.121, 0.111, 0.1534), "rear": (0.166, 0.231, 0.148)}
# Solo para el dibujo: largo de manivelas/palancas y separacion entre ejes de
# motor en el torso (todos a la altura de la cadera).
CRANK_LENGTH = 0.03
MOTOR_SPACING = 0.045
# Mismo suavizado elegido en servos/plot_one_cycle.py, para las velocidades pico.
SMOOTH_HARMONICS = 4

COLORS = {"hip": "#2a78d6", "knee": "#eb6834", "ankle": "#1baf7a"}
INK = "#0b0b0b"
INK_MUTED = "#52514e"
GRID = "#d9d8d4"


def fourier_velocity(x, dt, n_harmonics):
    """Derivada de una senal periodica (un ciclo exacto) tras un pasa-bajos de Fourier.

    Mismo filtro que fourier_smooth de servos/plot_one_cycle.py: conserva el
    armonico 0 y los primeros n_harmonics.
    """
    n = len(x)
    spectrum = np.fft.rfft(x)
    spectrum[n_harmonics + 1:] = 0
    omega = 2 * np.pi * np.fft.rfftfreq(n, d=dt)
    return np.fft.irfft(1j * omega * spectrum, n)


def leg_lengths(leg):
    return SEGMENT_LENGTHS["front" if leg.startswith("f") else "rear"]


def joint_to_motor(hip, knee, ankle):
    """Angulos articulares -> angulos de los motores del torso (orientacion absoluta de cada segmento)."""
    return hip, hip - knee, hip - knee + ankle


def motor_to_joint(m_hip, m_knee, m_ankle):
    """Inversa de joint_to_motor."""
    return m_hip, m_hip - m_knee, m_ankle - m_knee


def joint_to_motor_torque(tau_hip, tau_knee, tau_ankle):
    """Torques articulares (ejes del modelo MuJoCo) -> torque que debe dar cada motor.

    Trabajo virtual: tau_m = M^-T tau_q, con M la matriz de joint_to_motor. Los
    motores de rodilla y tobillo reaccionan contra el torso y no contra el
    segmento anterior, asi que la carga se reparte distinto que en
    accionamiento directo.
    """
    return tau_hip + tau_knee, -tau_knee - tau_ankle, tau_ankle


def stage_variables(hip, knee, ankle):
    """Parte variable del angulo interno de cada paralelogramo de la pata.

    Angulo interno = variable + offset constante de la palanca respecto a su
    segmento. 90 = transmision ideal; 0/180 = el paralelogramo se aplana
    (singular: puede invertirse y perder el control del segmento).
      knee        : muslo vs palanca de la tibia
      ankle_upper : muslo vs brazo superior del balancin de rodilla
      ankle_lower : tibia vs palanca del metatarso
    """
    return {"knee": -knee, "ankle_upper": ankle - knee, "ankle_lower": ankle}


def centered_offset(variable):
    """Offset de palanca [deg] que centra el angulo interno en +-90 sobre el ciclo.

    De las dos soluciones (separadas 180) devuelve la de |offset| <= 90: la
    palanca apunta hacia el lado del segmento que la lleva, como en el
    mecanismo de referencia (palanca por debajo de la rodilla).
    """
    mid = (np.max(variable) + np.min(variable)) / 2
    return (180 - mid) % 180 - 90


def stage_angles(hip, knee, ankle, offsets):
    """Angulo interno [deg, 0-180] de cada paralelogramo: angulo entre sus dos pares de lados."""
    variables = stage_variables(hip, knee, ankle)
    return {s: np.mod(variables[s] + offsets[s], 180) for s in STAGES}


def singularity_margin(stage_angle):
    """Distancia minima [deg] del angulo interno a 0/180."""
    folded = np.mod(stage_angle, 180)
    return float(np.min(np.minimum(folded, 180 - folded)))


def _direction(phi_deg):
    # Direccion de un segmento en el plano sagital (x adelante, z arriba).
    phi = np.radians(phi_deg)
    return np.stack([-np.sin(phi), -np.cos(phi)], axis=-1)


def leg_points(leg, hip, knee, ankle):
    """Rodilla, tobillo y pie en el plano sagital [m], con la cadera en el origen."""
    l1, l2, l3 = leg_lengths(leg)
    m_hip, m_knee, m_ankle = joint_to_motor(hip, knee, ankle)
    p_knee = l1 * _direction(m_hip)
    p_ankle = p_knee + l2 * _direction(m_knee)
    p_foot = p_ankle + l3 * _direction(m_ankle)
    return p_knee, p_ankle, p_foot


def motor_table(df):
    """Angulos de motor de las 4 patas, mismo formato de tiempo que el CSV de entrada."""
    out = pd.DataFrame({"time_s": df["time_s"]})
    for leg in LEGS:
        motors = joint_to_motor(*(df[f"{leg}_{j}_joint"].to_numpy() for j in JOINTS))
        for joint, values in zip(JOINTS, motors):
            out[f"{leg}_{joint}_motor"] = values
    return out


def leg_summary(df, leg, dt):
    joints = [df[f"{leg}_{j}_joint"].to_numpy() for j in JOINTS]
    motors = joint_to_motor(*joints)
    variables = stage_variables(*joints)
    offsets = {s: centered_offset(v) for s, v in variables.items()}

    def peak_speed(x):
        return float(np.abs(fourier_velocity(x, dt, SMOOTH_HARMONICS)).max())

    lines = [f"Pata {leg.upper()}  (L = {leg_lengths(leg)} m)"]
    lines.append(f"  {'motor':8s}{'centro':>8s}{'rango':>8s}{'|vel|max':>10s}   directo: rango |vel|max")
    for joint, m, q in zip(JOINTS, motors, joints):
        lines.append(
            f"  {joint:8s}{(m.max() + m.min()) / 2:8.1f}{np.ptp(m):8.1f}{peak_speed(m):10.0f}"
            f"   {np.ptp(q):14.1f}{peak_speed(q):9.0f}"
        )
    lines.append(f"  {'paralelogramo':14s}{'offset':>8s}{'angulo interno':>18s}{'margen':>8s}")
    for stage, angle in stage_angles(*joints, offsets).items():
        lines.append(
            f"  {stage:14s}{offsets[stage]:8.1f}{angle.min():9.1f} a {angle.max():6.1f}"
            f"{singularity_margin(angle):8.1f}"
        )
    return "\n".join(lines), offsets


def draw_mechanism(ax, leg, hip, knee, ankle, offsets, r=CRANK_LENGTH):
    m_hip, m_knee, m_ankle = joint_to_motor(hip, knee, ankle)
    p_hip = np.zeros(2)
    p_knee, p_ankle, p_foot = leg_points(leg, hip, knee, ankle)
    # Cada manivela y su palanca/brazo opuesto son paralelos (paralelogramo).
    u_knee = r * _direction(m_knee + offsets["knee"])
    u_upper = r * _direction(m_ankle + offsets["ankle_upper"])
    u_lower = r * _direction(m_ankle + offsets["ankle_lower"])
    # Motores en fila a la altura de la cadera, hacia el centro del cuerpo.
    inward = -1 if leg.startswith("f") else 1
    motor_knee = np.array([inward * MOTOR_SPACING, 0.0])
    motor_ankle = np.array([inward * 2 * MOTOR_SPACING, 0.0])

    def bar(a, b, color, lw, **kw):
        ax.plot([a[0], b[0]], [a[1], b[1]], color=color, linewidth=lw, solid_capstyle="round", **kw)

    bar(motor_ankle + [inward * 0.02, 0], [-inward * 0.02, 0], GRID, 10, zorder=0)  # torso
    for a, b in [(p_hip, p_knee), (p_knee, p_ankle), (p_ankle, p_foot)]:
        bar(a, b, INK_MUTED, 6, zorder=1)
    # Rodilla: motor -> manivela en cadera -> biela || muslo -> palanca en tibia.
    for a, b in [(motor_knee, motor_knee + u_knee), (motor_knee + u_knee, u_knee),
                 (p_hip, u_knee), (u_knee, p_knee + u_knee), (p_knee, p_knee + u_knee)]:
        bar(a, b, COLORS["knee"], 2, zorder=3)
    # Tobillo: motor -> manivela en cadera -> biela || muslo -> balancin en
    # rodilla -> biela || tibia -> palanca en metatarso.
    for a, b in [(motor_ankle, motor_ankle + u_upper), (motor_ankle + u_upper, u_upper),
                 (p_hip, u_upper), (u_upper, p_knee + u_upper), (p_knee, p_knee + u_upper),
                 (p_knee, p_knee + u_lower), (p_knee + u_lower, p_ankle + u_lower),
                 (p_ankle, p_ankle + u_lower)]:
        bar(a, b, COLORS["ankle"], 2, zorder=2)
    for point, color in [(p_hip, COLORS["hip"]), (motor_knee, COLORS["knee"]), (motor_ankle, COLORS["ankle"])]:
        ax.plot(*point, "s", color=color, markersize=10, markeredgecolor="white", markeredgewidth=2, zorder=4)
    for point in (p_knee, p_ankle):
        ax.plot(*point, "o", color="white", markersize=8, markeredgecolor=INK_MUTED, markeredgewidth=2, zorder=4)


def _style_axis(ax, title):
    ax.set_title(title, color=INK, fontsize=11, loc="left")
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.tick_params(colors=INK_MUTED, labelsize=9)
    for spine in ax.spines.values():
        spine.set_color(GRID)


def _direct_labels(ax, t, ends, min_gap_frac=0.06):
    """Etiqueta cada serie al final de su curva, separando las que quedan encimadas."""
    lo, hi = ax.get_ylim()
    min_gap = min_gap_frac * (hi - lo)
    placed = []
    for y, text in sorted(ends):
        if placed and y - placed[-1][0] < min_gap:
            y = placed[-1][0] + min_gap
        placed.append((y, text))
    for y, text in placed:
        ax.annotate(text, (t[-1], y), xytext=(6, 0), textcoords="offset points",
                    va="center", fontsize=9, color=INK_MUTED)


def build_figure(df, offsets_by_leg):
    t = df["time_s"].to_numpy()
    fig, axs = plt.subplots(2, 3, figsize=(16, 10), gridspec_kw={"width_ratios": [1.1, 1, 1]})
    for row, leg in enumerate(["fr", "br"]):
        joints = [df[f"{leg}_{j}_joint"].to_numpy() for j in JOINTS]
        motors = joint_to_motor(*joints)
        offsets = offsets_by_leg[leg]
        name = "delantera" if leg.startswith("f") else "trasera"

        ax = axs[row, 0]
        foot = leg_points(leg, *joints)[2]
        frame = int(np.argmin(foot[:, 1]))  # pose de apoyo: pie en su punto mas bajo
        ax.plot(foot[:, 0], foot[:, 1], color=INK_MUTED, linewidth=1, linestyle=":", label="trayectoria del pie")
        draw_mechanism(ax, leg, *(q[frame] for q in joints), offsets)
        ax.set_aspect("equal")
        ax.set_xlabel("x, adelante [m]", color=INK_MUTED)
        ax.set_ylabel("z [m]", color=INK_MUTED)
        ax.legend(loc="lower left", fontsize=8, frameon=False)
        _style_axis(ax, f"Pata {leg.upper()} ({name}), apoyo t = {t[frame]:.2f} s")

        ax = axs[row, 1]
        for joint, m in zip(JOINTS, motors):
            ax.plot(t, m, color=COLORS[joint], linewidth=2, label=f"motor {joint}")
        _direct_labels(ax, t, [(m[-1], joint) for joint, m in zip(JOINTS, motors)])
        ax.set_ylabel("Ángulo absoluto del segmento [°]", color=INK_MUTED)
        ax.legend(loc="best", fontsize=8, frameon=False)
        _style_axis(ax, "Motores del torso (0 = vertical, + = atrás)")

        ax = axs[row, 2]
        styles = {"knee": (COLORS["knee"], "-"), "ankle_upper": (COLORS["ankle"], "-"),
                  "ankle_lower": (COLORS["ankle"], "--")}
        angles = stage_angles(*joints, offsets)
        for stage, angle in angles.items():
            color, ls = styles[stage]
            ax.plot(t, angle, color=color, linestyle=ls, linewidth=2, label=stage)
        ax.axhline(90, color=INK_MUTED, linewidth=1, linestyle="--")
        for lo, hi in [(-90, 20), (160, 270)]:
            ax.axhspan(lo, hi, color=GRID, alpha=0.6, linewidth=0)
        ax.set_ylim(0, 180)
        _direct_labels(ax, t, [(a[-1], stage) for stage, a in angles.items()])
        ax.set_yticks([0, 20, 45, 90, 135, 160, 180])
        ax.set_ylabel("Ángulo interno [°]", color=INK_MUTED)
        ax.legend(loc="lower left", fontsize=8, frameon=False)
        _style_axis(ax, "Paralelogramos (90 = ideal, gris = cerca de singular)")
    for ax in axs[:, 1:].flat:
        ax.set_xlim(t[0], t[-1] * 1.18)
    for ax in axs[1, 1:]:
        ax.set_xlabel("Tiempo [s]", color=INK_MUTED)
    fig.suptitle("Rodilla y tobillo accionados desde el torso por paralelogramos", color=INK, fontsize=13)
    fig.tight_layout()
    return fig


def main():
    df = pd.read_csv(CSV_PATH)
    dt = float(np.median(np.diff(df["time_s"].to_numpy())))
    offsets_by_leg = {}
    for leg in LEGS:
        text, offsets_by_leg[leg] = leg_summary(df, leg, dt)
        print(text, end="\n\n")
    motor_table(df).to_csv(OUT_CSV, index=False, float_format="%.4f")
    print(f"Guardado: {OUT_CSV.name}")
    fig = build_figure(df, offsets_by_leg)
    fig.savefig(OUT_PNG, dpi=130)
    print(f"Guardado: {OUT_PNG.name}")
    plt.show()


if __name__ == "__main__":
    main()
