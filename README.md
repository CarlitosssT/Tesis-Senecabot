# Tesis SenecaBot

**SenecaBot** es un robot cuadrúpedo bioinspirado en la cabra. Este repositorio reúne todo el trabajo de tesis
sobre él, en dos partes que se complementan:

| Parte | Qué hay | Dónde corre |
|---|---|---|
| [`simulacion/`](simulacion/) | El robot en MuJoCo aprende a caminar imitando la marcha de una oveja capturada con MOCAP (aprendizaje por refuerzo, PPO). Encima de eso: selección de motores, dimensionamiento de eslabones y el rediseño de la pata (RR). | Linux / WSL2 con GPU NVIDIA |
| [`hardware/`](hardware/) | El robot físico (en el código aparece como **Eris**): firmware de la ESP32, control de servos, transmisión de las patas y un actuador MyActuator por CAN. | Windows o Linux, con la ESP32 por USB |

El puente entre las dos: la **marcha de referencia** que se genera en simulación
(`simulacion/seneca_loco/data/processed_data/trajectory_adapted.npz`) es la que se exporta a CSV y se reproduce
en el hardware (`hardware/servos/leg_actuator_angles_*.csv`).

> Este repo nace de la fusión de dos repositorios anteriores, con todo su historial de git:
> [Definicion-Senecabot](https://github.com/CarlitosssT/Definicion-Senecabot) → `simulacion/` y
> [Senecabot_mini](https://github.com/CarlitosssT/Senecabot_mini) → `hardware/`.
> Si un documento antiguo menciona una ruta que ya no existe, búscala en [`docs/rutas_antiguas.md`](docs/rutas_antiguas.md).

---

## ¿Por dónde empiezo?

| Si quieres… | Lee primero |
|---|---|
| Entender el proyecto en 10 minutos | Este README → [`docs/historia.md`](docs/historia.md) (qué se hizo y por qué) |
| No perderte con los nombres (RRR, RR, v140, FR/BR, …) | [`docs/glosario.md`](docs/glosario.md) |
| Trabajar con ángulos de las patas (simulación o servos) | [`docs/convenciones.md`](docs/convenciones.md) — **lectura obligatoria**, es la fuente de errores más común |
| Instalar y entrenar / evaluar agentes | [`simulacion/README.md`](simulacion/README.md) |
| Cambiar el entrenamiento o la recompensa | [`simulacion/CLAUDE.md`](simulacion/CLAUDE.md) y [`simulacion/seneca_loco/CLAUDE.md`](simulacion/seneca_loco/CLAUDE.md) |
| Seleccionar motores o rediseñar la pata | [`simulacion/geometry_opt/README.md`](simulacion/geometry_opt/README.md) |
| Ver los análisis (microsaltos, pendientes, pata RR) | [`simulacion/analisis/README.md`](simulacion/analisis/README.md) |
| Mover servos o el motor MyActuator | [`hardware/README.md`](hardware/README.md) |

---

## Mapa del repositorio

```
Tesis-Senecabot/
├── README.md                    ← estás aquí
├── docs/                        ← documentación transversal (historia, convenciones, glosario)
│
├── simulacion/                  ← todo lo que corre en MuJoCo (antes: Definicion-Senecabot)
│   ├── seneca_loco/             ←   proyecto base: modelo del robot, MOCAP → referencia, entrenamiento, agentes
│   ├── loco-mujoco-seneca/      ←   fork de LocoMuJoCo con los entornos SenecaBot y SenecaBotRR
│   ├── geometry_opt/            ←   motores, estructura y mecanismo de la pata RR; genera el XML RR
│   ├── analisis/                ←   estudios puntuales: microsaltos, pendientes, pata RR
│   ├── catalogo_myactuator/     ←   catálogo de motores MyActuator (datos de entrada de geometry_opt)
│   └── requirements.txt         ←   versiones exactas del entorno (Python 3.12, CUDA 12)
│
└── hardware/                    ← robot físico (antes: Senecabot_mini)
    ├── firmware/                ←   sketches de Arduino para la ESP32 (servos, WiFi, sensores)
    ├── servos/                  ←   notebooks de control de los 12 servos y CSV de marcha
    ├── leg_transmission/        ←   transmisión por paralelogramos: ángulos de marcha → ángulos de motor
    ├── myactuator_esp32/        ←   librería, GUI e identificación del motor MyActuator por CAN
    └── requirements.txt
```

Cada carpeta importante tiene su propio `README.md`. Los archivos `CLAUDE.md` son notas de contexto pensadas
para retomar el trabajo con Claude Code, pero también sirven a personas: contienen las decisiones y las
lecciones aprendidas con más detalle que los README.

---

## Inicio rápido

**Simulación** (Linux/WSL2 con GPU; guía completa, incluida la de Windows, en [`simulacion/README.md`](simulacion/README.md)):

```bash
git clone https://github.com/CarlitosssT/Tesis-Senecabot.git ~/Tesis-Senecabot
cd ~/Tesis-Senecabot
python3.12 -m venv .venv && source .venv/bin/activate
pip install -r simulacion/requirements.txt
pip install -e simulacion/loco-mujoco-seneca -e simulacion/seneca_loco
export SENECA_HOME="$HOME/Tesis-Senecabot/simulacion/seneca_loco"
cd simulacion/seneca_loco && python main.py        # menú interactivo
```

**Hardware** (detalles en [`hardware/README.md`](hardware/README.md)):

```bash
cd hardware
pip install -r requirements.txt
python leg_transmission/leg_transmission.py         # regenera los ángulos de motor desde el ciclo de marcha
python myactuator_esp32/gui.py                      # GUI del motor MyActuator
```

---

## Estado actual (noviembre 2026)

- **Simulación — frente activo: pata RR.** Se quitó la cadera; rodilla y tobillo se mueven desde el torso con dos
  mecanismos de barras. Agente elegido: `simulacion/seneca_loco/artifacts/trained_agents_rr/2026-11-04/17-37-28/PPOJax_final.pkl`
  (0 caídas, 1.16 m/s). Pendiente: las patas traseras rebotan ~2 cm al aterrizar.
- **Hardware:** transmisión por paralelogramos calculada (falta calibrar offsets y signos físicos) y modelo del
  MyActuator identificado en modo 0x400: G(s) = 1.013 / (0.0146 s + 1) · e^(−0.005 s) (falta la zona muerta y el
  compensador).

La lista completa de pendientes está al final de [`docs/historia.md`](docs/historia.md).

---

## Créditos y licencias

El proyecto base `simulacion/seneca_loco/` y el fork `simulacion/loco-mujoco-seneca/` provienen de la tesis de
pregrado de **Nicolás Andrade Manrique** (*"Model & Locomotion Dynamics of a Goat Bio-Inspired Quadruped Robot
Through Reinforcement Learning"*, Ingeniería Mecánica, Universidad de los Andes, 2026; asesor Jonathan Camargo
Leyva, PhD), construida sobre [LocoMuJoCo](https://github.com/robfiras/loco-mujoco) (Al-Hafez et al.). Ambas
carpetas conservan su licencia MIT original (archivo `LICENSE` en cada una). La lista completa de créditos y la
cita de LocoMuJoCo están en [`simulacion/README.md`](simulacion/README.md#créditos).

El resto del código (continuación de Carlos Andrés Tovar, Uniandes, desde septiembre de 2026) todavía no tiene
una licencia definida.
