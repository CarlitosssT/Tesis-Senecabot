# CLAUDE.md — Contexto del proyecto Eris (tesis)

Este archivo es para que otra instancia de Claude retome el proyecto con todo el contexto. Usuario: Carlos Andrés Tovar (Uniandes). **Habla en español**; responder en español, directo y conciso. Entorno: Windows 11, PowerShell/Git Bash, VSCode. Rama `master`.

## Qué es el proyecto
Tesis sobre un **robot cuadrúpedo tipo cabra ("Eris" / senecabot)**. Una pata robótica reproduce la marcha de una cabra real (trayectorias articulares adaptadas, simuladas en MuJoCo). Este repo (`Tesis/Python`) contiene el **hardware real y su control**: servos, transmisión mecánica de las patas, y un actuador MyActuator por CAN. Cada pata tiene 3 articulaciones: hip (cadera), knee (rodilla), ankle (tobillo/metatarso). Patas: `fr`, `fl`, `br` (a veces `rr`), `bl` (`rl`).

**Fuera de este repo** (no existe aquí, pero se referencia):
- `Tesis/workspace/seneca_loco/simulation/assets/xml/senecabot_loco.xml` — modelo MuJoCo.
- `Tesis/workspace/seneca_loco/data/processed_data/trajectory_adapted.npz` — origen de los CSV de marcha.
- `PCR_GP-main/Arduino/motor_control/motor_control.ino` — firmware ESP32↔CAN del motor MyActuator.

## Reglas / preferencias del usuario (importantes)
1. **No trabajar en `servos/`**: el trabajo de transmisión va en `leg_transmission/`; de `servos/` solo se leen los CSV de marcha.
2. Las respuestas del usuario suelen ser cortas ("hazlo", "haz el siguiente paso"): ejecutar, no re-preguntar lo que ya está claro.
3. Para cinemática/animaciones, el usuario valora: gráficas legibles, animaciones fluidas, mostrar la señal enviada, tablas-resumen de eslabones.
4. Memoria persistente de Claude en `C:\Users\andre\.claude\projects\c--Users-andre-OneDrive-Documents-Tesis-Python\memory\` (tres notas: signos de servos, CSV=MuJoCo, transmisión por paralelogramos). Su contenido está resumido abajo.
5. Commits con `Co-Authored-By` de Claude al final. Solo commitear/pushear si el usuario lo pide. `tesis/` (venv) y `.claude/` están en `.gitignore`.

## Convenciones críticas (fuente frecuente de errores)

### A. Los CSV de marcha usan la convención MuJoCo
`servos/leg_actuator_angles_trajectory_adapted.csv` y los `*_one_cycle*.csv` son exactamente `rad2deg(qpos[:, 7:19])` del `.npz` anterior (verificado 2026-10-24, diff máx 7e-6°). Columnas `<pata>_<hip|knee|ankle>_joint`, paso de 5 ms, ~95 repeticiones de un mismo ciclo (el one-cycle tiene 127 filas).

FK sagital validada (coincide con `xpos` de MuJoCo a 7 µm en las 4 patas): **cabeceo absoluto** alrededor de +y, 0 = segmento colgando vertical, + = rota hacia atrás.
- muslo = hip; tibia = hip − knee (eje de rodilla −y); metatarso = hip − knee + ankle.
- Dirección (x adelante, z arriba) = (−sin φ, −cos φ).
- Longitudes muslo/tibia/metatarso [m]: delantera 0.121 / 0.111 / 0.1534; trasera 0.166 / 0.231 / 0.148.

⚠️ `plot_leg_actuator_angles.ipynb` usa una convención *adivinada* ("servo 90 = recto, offset base −90") que **NO** reproduce la pose simulada. Para cualquier cinemática desde el CSV usar la de MuJoCo (ver `leg_transmission/leg_transmission.py`). (`servos/phase_match_*.py` también heredan el offset −90; tenerlo en cuenta.)

### B. Servos físicos: ángulo biomecánico vs. crudo
Las 12 servos (PCA9685) tienen 90° crudo = reposo horizontal, pero patas delanteras y traseras están **montadas en espejo** (confirmado a mano 2026-09-24). `JOINT_SIGN` (en `eris_servo_control.ipynb`):

| Pata | hip | knee | ankle |
|---|---|---|---|
| FR, FL | +1 | −1 | +1 |
| RR/BR, RL/BL | −1 | +1 | −1 |

`raw = 90 + signo · bio`. `move_joint()` recibe ángulo biomecánico (convierte con `bio_to_raw_angle` / `raw_to_bio_angle`); `move_servo(canal, ángulo)` es crudo. Esta convención es **distinta** de la de MuJoCo (A).

### C. Rediseño con transmisión por paralelogramos (desde 2026-10-24)
Todos los actuadores van en el torso, a la misma altura (espacio y tamaño). Cadera en accionamiento directo; rodilla y tobillo por paralelogramos (referencia: CAD con servos MG996R; rodilla = manivela en eje de cadera → biela ∥ muslo → palanca en tibia; tobillo = dos en serie con balancín libre en el eje de rodilla). Un paralelogramo copia orientaciones ⇒ cada motor fija el ángulo **absoluto** de su segmento:

```
m_hip = hip,   m_knee = hip − knee,   m_ankle = hip − knee + ankle   (convención MuJoCo)
```
Implementado y verificado numéricamente en `leg_transmission/leg_transmission.py` → `leg_motor_angles_one_cycle.csv` (columnas `<pata>_<art>_motor`) y `leg_transmission.png`; visor en `leg_transmission.ipynb`. Pendiente: offsets/signos físicos de los cuernos de servo (calibrar como en B).

## Mapa de módulos

### `servos/` — servos PWM, marcha, estimación de fase
- Hardware: ESP32 + PCA9685 (I2C0 SDA=21, SCL=22, dir 0x40) + INA219 (I2C1 `Wire1` SDA=32, SCL=33, dir 0x40, bus aparte para no interferir).
- Firmware (en la raíz del repo): `eris_servo_ina/` (16 servos + INA219, protocolo texto `X`/`Y`/`INA`/`S_F`/`S_TIME`/`INFO`/`ON`/`OFF`, 921600 baud); `eris_servo_wifi/` (sin INA; ESP32 crea su propia red SoftAP, IP 192.168.4.1, TCP 8888, también Serial); `esp32_servo_control/` (3 servos pines 13/26/15, `a1,a2,a3[,ms]`, 115200); `test_ina219/`; `test_bmi160/` (IMU BMI160, I2C propio SDA=25, SCL=26, sin librerías).
- Notebooks: `eris_servo_control.ipynb` (principal: movimiento, captura de corriente en vivo con autoajuste de escala en mA, export CSV), `send_servo_command.ipynb`, `send_serial.ipynb` (comandos por cable, mismo .ino), `send_csv_wifi.ipynb` (CSV por WiFi: columna 1 → canal 0 del PCA, columna 2 → canal 1, …), `plot_leg_actuator_angles.ipynb`.
- `plot_one_cycle.py`: visor ángulo/velocidad/aceleración de un ciclo + suavizado de Fourier (armónicos 4/6/8/12) → `..._smooth_n4.csv`.
- `phase_match_sim.py` / `phase_match_live.py`: prototipo de viabilidad pedido por el usuario — leer un IMU en el ankle de la cabra real y hacer que la pata robot reproduzca la marcha estimando la **fase del ciclo** (con θ, ω, α del IMU) y eligiendo hip/knee de la tabla predefinida. Conclusiones de la simulación: la mejor config fue θ+ω, modo `recover` (ventana + salto global si es mucho mejor); se calibra en la misma pose para eliminar orientación global; se discutió un filtro (complementario) para el IMU. Se corre como módulo desde la raíz (`python -m servos.phase_match_sim`).
- Nota: caídas de tensión pueden resetear la calibración del INA219 (corriente leída = 0); el firmware la rearma cada 2 s.

### `leg_transmission/` — mecanismos y transmisión
- `leg_transmission.py`: transmisión por paralelogramos (ver C). Lee `servos/leg_actuator_angles_one_cycle.csv`.
- `Mecanismo.ipynb`, `MecanismoV2.ipynb`, `Version2.ipynb`: **exploración previa/alternativa** de un mecanismo **RRR (muslo, tibia, metatarso) con dos mecanismos RR acoplados** (actuadores A1 en cadera directa; A2, A3 vía RR con eslabones `l1–l4` acoplados a la tibia (fracción `f1`) y al metatarso (`f2`)). Modelo simbólico con sympy (ecuaciones de cierre de lazo, 3 thetas actuadores y alphas pasivos), dimensionado/optimización de `l1..l4` y `f1,f2` para cubrir un rango de movimiento amplio (no solo la marcha), cinemática inversa de los RR por ley de cosenos, restricciones: sin superposición de elementos, `dx`/`dy` entre actuadores no pueden bajar de los mínimos (≈33.9 mm), separación de actuadores d1=84 mm, d2=97 mm (supuesto igual en pata trasera); animaciones `mecanismo_*.gif`; en sesiones se pidió también reflejarlo en el XML de MuJoCo (cápsulas del ancho de las patas, con los dos RR visibles) y que el brazo del RRR apunte hacia abajo. *Nota: esa parte la leí solo por los títulos/markdown de los notebooks; verificar el estado antes de basarse en ella.* El diseño más reciente y vigente es el de paralelogramos (C).

### `myactuator_esp32/` y `motor_sequence.py` — motor MyActuator por CAN
- La ESP32 traduce comandos de texto (`p <grados> <dps>`, `e`, `s`, …) a tramas CAN; el PC solo usa `pyserial`.
- `myactuator_esp32.py`: clase `MyActuator(port, motor_id)`: `move_to`, `read_angle` (0.01°), `play(t, ángulos)` (trayectoria ≤1000 puntos se sube a la ESP32, que la reproduce con su reloj y lee encoder tras cada comando; modos `MODE_POSITION` 0xA4 y `MODE_MOTION` 0x400 con lazo kp/kd). Límites: ±360°, 300 dps, 600 dps en trayectoria, aceleración 100–60000 dps².
- `gui.py` (tkinter+matplotlib): posición manual, deseado vs medido en vivo, carga de CSV, pestaña "Última ejecución", exporta CSV. `tracking.py` (lectura CSV, `analyse`, `build_reference`, `save_log`), `track_csv.py` (CLI: RMS, error máx, retardo; CSV por defecto `leg_transmission/leg_motor_angles_one_cycle.csv`, ej. columna `fr_hip_motor`). Hay un registro de ejemplo `track_fr_hip_motor_20261106_103421.*`.
- `sysid/`: `make_sysid_signals.py` genera señales de identificación (1000 puntos, 100 Hz, empiezan/terminan en 0°): chirp 0.2–15 Hz, multisine (24 senoidales, fases de Schroeder, 2 periodos), steps (±1,2,3,5°), ramp (2 dps). Se cargan en `gui.py` con escala 1, ciclos 1, offset 0, 600 dps. `visualizar_sysid.ipynb` grafica `sysid_*.csv` contra las `ejecucion_*.csv` reales **y contiene la identificación del modelo comando→posición** (ver abajo).
  - **Modos:** los `ejecucion_*_0x400.csv` son de `MODE_MOTION` (0x400); los demás, del modo posición 0xA4 ("modo A", mucho más lento). Typo conocido: `ejecucion_cihrp_0x400.csv` es el chirp 0x400.
  - **Identificación (2026-11-10, en el notebook, autocontenida):** modelos 1.er/2.º/3.er orden + retardo `Td`, ajustados por mínimos cuadrados no lineales para los grupos `TODOS`, `A` y `0x400`. El retardo se aplica a la **salida** con interpolación (aplicarlo a la entrada sobre rejilla de 1 ms lo dejaba cuantizado y el optimizador nunca lo movía). Incluye tabla comparativa, Bode/escalón y perfil de `Td`. Caché en `tf_resultados.json` y `tf_perfil.json` (`REFIT = True` para recalcular); export en `tf_por_modo.json`.
  - **Decisión del usuario: se usa el modo 0x400.** Modelo vigente: `G(s) = 1.013 / (0.0146 s + 1) · e^(−0.005 s)` (ajuste 86–90 % en sus archivos). `Td` ∈ [0, 5] ms es poco identificable con 100 Hz; lo robusto es K≈1 y τ≈15 ms. Se descartó el modelo `TODOS` (τ≈44 ms) porque mezcla el modo A (τ≈49 ms). El 2.º/3.er orden no mejora al 1.º (retardo y orden se compensan).
  - **Limitaciones sin modelar:** error estático ~0.3° en escalones (zona muerta/fricción) y saturación de velocidad/aceleración en el chirp del modo A.
- `motor_sequence.py`: secuencias `angulo[:dps[:pausa]]` o CSV `angulo,velocidad,pausa`.

## Estado del repo (git)
Un solo commit (`ebf7ea7`, "Initial commit"). El árbol de trabajo tiene reorganización sin commitear: los archivos originales de la raíz figuran como borrados porque se movieron a `servos/`; `eris_servo_wifi/`, `leg_transmission/`, `myactuator_esp32/`, `servos/`, `test_bmi160/`, `motor_sequence.py` y `README.md`/`CLAUDE.md` están sin trackear. `servos/fr_leg_animation.gif` pesa ~25 MB (considerar LFS/excluir). `README.md` (raíz) es la descripción general; `servos/README.md` detalla el módulo de servos.

## Comandos útiles
```bash
pip install -r servos/requirements.txt
python leg_transmission/leg_transmission.py                       # regenera ángulos de motor
python motor_sequence.py --port COM3 0 90 180:60 -45
python myactuator_esp32/track_csv.py --port COM3 --column fr_hip_motor
python myactuator_esp32/gui.py
python myactuator_esp32/sysid/make_sysid_signals.py --plot
python -m servos.phase_match_sim      # desde la raíz
```
Ajustar `PORT` (COMx) en notebooks/scripts. Al leer notebooks con Python en Windows usar `encoding='utf-8'` y `PYTHONIOENCODING=utf-8` (la consola cp1252 falla con flechas/acentos).

## Pendientes / ideas abiertas
- Calibrar offsets y signos físicos de la nueva transmisión por paralelogramos.
- Corregir (o marcar como obsoletos) los cálculos del notebook de plot/phase_match que usan la convención "90 = recto".
- MyActuator (modo 0x400): modelo lineal ya identificado; falta modelar zona muerta/fricción, validarlo con trayectorias reales (ej. `fr_hip_motor`) y diseñar prefiltro/compensador (τ≈15 ms).
- Integración IMU-ankle → pata robot en hardware real (hoy solo simulada); `test_bmi160` es el primer paso.
- Decidir sobre el gif de 25 MB y hacer commit de la reorganización.
