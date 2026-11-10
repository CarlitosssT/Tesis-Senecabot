# Convenciones

Reglas que hay que conocer antes de tocar ángulos, archivos de marcha o código compartido. Casi todos los errores
que se cometieron en el proyecto vinieron de mezclar estas convenciones.

---

## 1. Nombres de patas y articulaciones

| Abreviatura | Pata | Notas |
|---|---|---|
| `fr` | delantera derecha (*front right*) | |
| `fl` | delantera izquierda (*front left*) | |
| `br` | trasera derecha (*back right*) | en algunos notebooks de `hardware/` aparece como `rr` (*rear right*) |
| `bl` | trasera izquierda (*back left*) | en algunos notebooks de `hardware/` aparece como `rl` (*rear left*) |

> ⚠️ **"RR" significa dos cosas distintas.** En `hardware/servos/` puede ser la pata *rear right* (= `br`).
> En `simulacion/` y en los notebooks de mecanismos es el **mecanismo de barras RR** (dos eslabones con dos
> articulaciones de revolución) y el **modelo de pata RR**. Si ves `rr1` / `rr2` / `SenecaBotRR` / `_rr.xml`, es el
> mecanismo, no la pata.

**Pata RRR** (modelo original, `senecabot_loco.xml`): tres articulaciones por pata, `hip` (cadera, mueve el
muslo), `knee` (rodilla, mueve la tibia) y `ankle` (tobillo, mueve el metatarso). Nombres en MuJoCo:
`<pata>_<articulación>_joint`, p. ej. `fr_knee_joint`. 12 actuadores.

**Pata RR** (rediseño, `senecabot_loco_rr.xml`): sin muslo ni cadera. Rodilla y tobillo son pasivos y los mueven
dos manivelas en el torso, `<pata>_rr1` (tibia) y `<pata>_rr2` (metatarso). 8 actuadores.

---

## 2. Ángulos: hay tres convenciones y no son intercambiables

### A. Convención MuJoCo (simulación y CSV de marcha)

Es la del `qpos` del modelo. La usan las referencias `.npz`, los agentes y los CSV
`hardware/servos/leg_actuator_angles_*.csv` (que son exactamente `rad2deg(qpos[:, 7:19])` de
`simulacion/seneca_loco/data/processed_data/trajectory_adapted.npz`, verificado el 2026-10-24; el script que hizo esa
exportación no está en el repo).

- **Cabeceo absoluto** alrededor de +y: 0 = segmento colgando vertical, positivo = rota hacia atrás.
- El eje de la rodilla es −y, por eso entra con signo menos al pasar a ángulos absolutos de segmento:

  ```
  muslo     = hip
  tibia     = hip − knee
  metatarso = hip − knee + ankle
  ```

- Dirección de un segmento (x adelante, z arriba) = (−sin φ, −cos φ).
- Longitudes muslo / tibia / metatarso [m]: delantera `0.121 / 0.111 / 0.1534`; trasera `0.166 / 0.231 / 0.148`.
- CSV: columnas `<pata>_<hip|knee|ankle>_joint` en grados, paso de 5 ms (200 Hz). El archivo `*_one_cycle.csv`
  tiene un solo ciclo (127 filas); `*_trajectory_adapted.csv` repite el mismo ciclo ~95 veces.

La implementación de referencia de esta cinemática (validada contra MuJoCo a 7 µm) está en
`hardware/leg_transmission/leg_transmission.py`.

### B. Convención de los servos físicos (`hardware/servos/`)

Los 12 servos (PCA9685) tienen 90° crudo = reposo horizontal, pero las patas delanteras y traseras están
**montadas en espejo**. Por eso se distingue:

- **Ángulo biomecánico**: 0 = reposo, positivo = eslabón hacia arriba, mismo sentido en las 4 patas. Lo usan
  `move_joint()` y las columnas `<pata>_<eslabón>` (p. ej. `FR_hip`).
- **Ángulo crudo**: lo que recibe el servo. Lo usan `move_servo(canal, ángulo)` y las columnas `chN`.

`crudo = 90 + signo · bio`, con `JOINT_SIGN` (definido en `eris_servo_control.ipynb`):

| Pata | hip | knee | ankle |
|---|---|---|---|
| FR, FL (delanteras) | +1 | −1 | +1 |
| BR/RR, BL/RL (traseras) | −1 | +1 | −1 |

> ⚠️ `hardware/servos/plot_leg_actuator_angles.ipynb` y `phase_match_*.py` usan una convención *supuesta*
> ("servo 90 = recto, offset −90") que **no** reproduce la pose simulada. Para cinemática a partir de los CSV usa
> la convención A.

### C. Ángulos de motor con transmisión por paralelogramos (`hardware/leg_transmission/`)

Un paralelogramo copia orientaciones, así que cada motor del torso fija el **ángulo absoluto** de su segmento
(convención A):

```
m_hip = hip      m_knee = hip − knee      m_ankle = hip − knee + ankle
```

Salida: `leg_motor_angles_one_cycle.csv`, columnas `<pata>_<hip|knee|ankle>_motor`. Falta calibrar los offsets y
signos físicos de los cuernos de servo (como en B).

---

## 3. Archivos que no se deben editar ni borrar

| Archivo | Por qué |
|---|---|
| `simulacion/seneca_loco/simulation/assets/xml/senecabot_loco_rr.xml` | Lo genera `python -m geometry_opt.make_rr_xml`. Edítalo a mano y se pierde en la próxima generación. |
| `simulacion/seneca_loco/data/processed_data/*.bak.npz` | Referencias anteriores conservadas a propósito: los agentes viejos se comparan contra ellas. |
| `simulacion/seneca_loco/artifacts/` | Agentes entrenados y sus registros (Hydra, wandb). Son el resultado del trabajo. |

Cada cambio de geometría en `make_rr_xml.py` invalida los agentes RR anteriores y obliga a regenerar las
referencias RR.

---

## 4. Cómo documentar los cambios

| Si cambias… | Documenta en… |
|---|---|
| El fork `simulacion/loco-mujoco-seneca/` | Comentario `[SENECA LOCAL CHANGE]` en el código **y** entrada fechada en `loco-mujoco-seneca/SENECA_LOCAL_CHANGES.md` (la más nueva arriba). |
| `simulacion/seneca_loco/` | Entrada fechada en la sección *Change Log* de `seneca_loco/CLAUDE.md` (la más nueva arriba). |
| `simulacion/geometry_opt/` | `geometry_opt/README.md`. |
| Cualquier cosa que cambie el panorama general | [`historia.md`](historia.md) y, si aplica, los README. |

- Los términos nuevos de recompensa deben tener un valor por defecto que deje el comportamiento anterior intacto
  (p. ej. `= 0`), para que los agentes y configuraciones viejos sigan siendo reproducibles.
- **Idioma:** la documentación general está en español; los change logs técnicos (`seneca_loco/CLAUDE.md`,
  `SENECA_LOCAL_CHANGES.md`, `geometry_opt/README.md`) y el código heredado están en inglés.
- Las rutas de los change logs anteriores al 2026-11-10 usan la estructura antigua; ver
  [`rutas_antiguas.md`](rutas_antiguas.md).

---

## 5. Sobre el repositorio

- Se versionan también los resultados: agentes (`.pkl`), referencias (`.npz`), figuras, grabaciones y registros
  de wandb. Por eso el repo pesa ~600 MB. GitHub rechaza archivos de más de 100 MB.
- `__pycache__/`, entornos virtuales (`.venv/`, `venv/`, `tesis/`) y `.claude/` están en `.gitignore`.
- Los sketches de Arduino deben estar en una carpeta con el mismo nombre que el `.ino` (requisito del IDE).
