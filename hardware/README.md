# Hardware — robot físico (Eris)

Control, transmisión y calibración de las patas del robot físico de SenecaBot (en el código aparece como
**Eris**). Antes era el repositorio `Senecabot_mini`; ahora es la parte de hardware de
[`Tesis-Senecabot`](../README.md). La simulación está en [`../simulacion/`](../simulacion/).

Tres líneas de trabajo:

1. **Servos PWM** (`servos/`): control de las 12 articulaciones con servos vía ESP32 + PCA9685, medición de
   corriente (INA219) y análisis/animación de trayectorias de marcha.
2. **Transmisión por paralelogramos** (`leg_transmission/`): diseño cinemático para mover rodilla y tobillo desde
   el torso, y conversión del ciclo de marcha a ángulos de motor.
3. **Actuador MyActuator por CAN** (`myactuator_esp32/`): control de posición de un motor MyActuator a través de
   una ESP32, seguimiento de trayectorias, GUI e identificación de sistema.

> Antes de trabajar con ángulos lee [`../docs/convenciones.md`](../docs/convenciones.md): los CSV de marcha usan la
> convención de MuJoCo y los servos otra distinta.

## Estructura

```
hardware/
├── requirements.txt                 # dependencias de Python de toda esta carpeta
├── firmware/                        # sketches de Arduino para la ESP32
│   ├── eris_servo_ina/              #   principal: 16 servos + corriente INA219 (Serial)
│   ├── eris_servo_wifi/             #   16 servos por WiFi (SoftAP + TCP) y Serial
│   ├── esp32_servo_control/         #   3 servos con pines directos (una pata, pruebas)
│   ├── test_ina219/                 #   prueba del sensor de corriente
│   └── test_bmi160/                 #   prueba del IMU BMI160 (I2C directo)
├── servos/                          # control de servos, marcha y estimación de fase
│   ├── eris_servo_control.ipynb     #   notebook principal (16 canales + INA219)
│   ├── send_servo_command.ipynb     #   banco de pruebas de 3 servos
│   ├── send_serial.ipynb, send_csv_wifi.ipynb   # envío de CSV por Serial / WiFi
│   ├── plot_leg_actuator_angles.ipynb  # análisis y animación de la pata FR
│   ├── plot_one_cycle.py            #   visor de un ciclo + suavizado de Fourier
│   ├── phase_match_sim.py           #   prototipo de estimación de fase con IMU (simulado)
│   ├── phase_match_live.py          #   versión en vivo del prototipo anterior
│   └── leg_actuator_angles_*.csv    #   trayectorias articulares (convención MuJoCo)
├── leg_transmission/                # transmisión torso → pata
│   ├── leg_transmission.py          #   ángulos de motor + figura
│   ├── leg_motor_angles_one_cycle.csv   # salida: ángulos de motor, 1 ciclo
│   ├── leg_transmission.ipynb       #   visor de la transmisión por paralelogramos
│   ├── Mecanismo*.ipynb, Version2.ipynb # exploración del mecanismo de barras RR (ver abajo)
│   └── *.png, *.gif                 #   figuras y animaciones
└── myactuator_esp32/                # motor MyActuator por CAN
    ├── myactuator_esp32.py          #   clase MyActuator (pyserial)
    ├── gui.py                       #   interfaz gráfica (tkinter + matplotlib)
    ├── tracking.py                  #   lectura de CSV y comparación enviado vs. medido
    ├── track_csv.py                 #   seguimiento de una columna de CSV (CLI)
    ├── motor_sequence.py            #   secuencia simple de ángulos (CLI)
    ├── track_fr_hip_motor_*.csv/png #   ejemplo de registro de una ejecución
    └── sysid/                       #   señales y registros para identificación de sistema
```

## De dónde salen los datos de marcha

Los CSV `servos/leg_actuator_angles_*.csv` son la marcha de referencia de la simulación,
`../simulacion/seneca_loco/data/processed_data/trajectory_adapted.npz`, convertida a grados
(`rad2deg(qpos[:, 7:19])`). Si la referencia de la simulación cambia, hay que regenerar estos CSV y después correr
`leg_transmission/leg_transmission.py`.

## Uso rápido

Desde `hardware/`:

```bash
pip install -r requirements.txt

# Regenerar los ángulos de motor desde el ciclo de marcha
python leg_transmission/leg_transmission.py

# Mover el motor MyActuator a una secuencia de ángulos (ángulo[:dps[:pausa_s]])
python myactuator_esp32/motor_sequence.py --port COM3 0 90 180:60 -45

# Seguir una columna del ciclo y comparar con el encoder
python myactuator_esp32/track_csv.py --port COM3 --column fr_hip_motor

# GUI del motor
python myactuator_esp32/gui.py

# Señales de identificación
python myactuator_esp32/sysid/make_sysid_signals.py --plot

# Prototipo de estimación de fase (como módulo, por el import `servos.*`)
python -m servos.phase_match_sim
python -m servos.phase_match_live
```

Para los notebooks, ajusta `PORT` a tu puerto serie (`COMx` en Windows, `/dev/ttyUSBx` en Linux).

**Firmware:** Arduino IDE con soporte para ESP32 y las librerías `Adafruit PWM Servo Driver Library`, `INA219`
(Rob Tillaart) y/o `ESP32Servo` según el sketch. Abre la carpeta del sketch dentro de `firmware/`.

## Módulos

### 1. Servos PWM (`servos/`, `firmware/`)

**Hardware:** ESP32 + PCA9685 (16 canales, I2C0: SDA=21, SCL=22) + INA219 (I2C1: SDA=32, SCL=33, bus aparte para
no interferir con el PCA9685). 12 servos en uso (hip/knee/ankle × 4 patas).

| Sketch (`firmware/`) | Función |
|---|---|
| `eris_servo_ina/` | Firmware principal. 16 servos + corriente INA219, protocolo de texto (`X`, `Y`, `INA`, `S_F`, `S_TIME`, `INFO`, `ON/OFF`). Baud 921600. Lo usa `eris_servo_control.ipynb`. |
| `eris_servo_wifi/` | Igual sin INA219; la ESP32 crea su propia red (SoftAP, IP `192.168.4.1`, TCP `8888`) y también acepta Serial. Lo usa `send_csv_wifi.ipynb`. |
| `esp32_servo_control/` | 3 servos (pines 13, 26, 15), formato `a1,a2,a3[,duración_ms]`, rampa interpolada. Baud 115200. Lo usa `send_servo_command.ipynb`. |
| `test_ina219/` | Verifica que el INA219 responde en el bus (útil si aparece "INA not found"). |
| `test_bmi160/` | Lee el IMU BMI160 por I2C propio (SDA=25, SCL=26), sin librerías. |

- `eris_servo_control.ipynb`: conexión, movimiento por canal/pata/trayectoria CSV, captura de corriente en vivo con
  exportación a CSV. Mapeo de canales en `LEG_JOINT_CHANNELS`.
- `plot_one_cycle.py`: ángulo/velocidad/aceleración de un ciclo y suavizado con armónicos de Fourier (4, 6, 8, 12)
  → `leg_actuator_angles_one_cycle_smooth_n4.csv`.
- `phase_match_sim.py` / `phase_match_live.py`: prototipo que estima la **fase del ciclo** de la pata FR a partir de
  un IMU en el tobillo (ángulo, velocidad, aceleración simulados) y elige hip/knee de la trayectoria predefinida.
  La mejor configuración fue θ+ω en modo `recover`.

Nota: el INA219 puede perder su calibración con caídas de tensión (la corriente leída queda en 0); el firmware la
rearma cada 2 s.

### 2. Transmisión (`leg_transmission/`)

`leg_transmission.py` lee `servos/leg_actuator_angles_one_cycle.csv`, calcula los ángulos de motor de torso para
cada pata y escribe `leg_motor_angles_one_cycle.csv` (columnas `<pata>_<hip|knee|ankle>_motor`) y
`leg_transmission.png`. La cadera es de accionamiento directo; rodilla y tobillo van por paralelogramos (uno para
la rodilla, dos en serie para el tobillo), así que cada motor fija el ángulo absoluto de su segmento.

Los notebooks `Mecanismo.ipynb`, `MecanismoV2.ipynb` y `Version2.ipynb` exploran otra idea: pata RRR con la cadera
directa y tibia y metatarso movidos por **dos mecanismos de barras RR**. `MecanismoV2.ipynb` se portó a `../simulacion/geometry_opt/linkage.py`, que es
la base del rediseño de la pata RR en simulación. La comparación de los dos diseños está en
[`../docs/historia.md`](../docs/historia.md#dos-diseños-de-transmisión-de-la-pata).

### 3. Motor MyActuator por CAN (`myactuator_esp32/`)

La ESP32 corre el firmware `motor_control.ino` (**no incluido en este repo**; vive en el proyecto `PCR_GP-main`)
que traduce comandos de texto a tramas CAN. No se necesita adaptador USB-CAN, solo `pyserial`.

- **`myactuator_esp32.py`**: clase `MyActuator(port, motor_id)`: `move_to`, `read_angle` (0.01°),
  `play(tiempos, ángulos)`. La trayectoria (hasta 1000 puntos) se sube a la ESP32, que la reproduce con su propio
  reloj y lee el encoder tras cada comando. Modos `MODE_POSITION` (0xA4) y `MODE_MOTION` (0x400, lazo kp/kd).
  Límites: ±360°, 300 dps (comandos), 600 dps (trayectoria), aceleraciones 100–60000 dps².
- **`gui.py`**: posición manual, gráfica en vivo deseado vs. medido, carga de CSV, reproducción, pestaña "Última
  ejecución" y exportación a CSV.
- **`track_csv.py`**: envía una columna de `leg_transmission/leg_motor_angles_one_cycle.csv` y reporta RMS, error
  máximo y retardo. **`tracking.py`**: utilidades compartidas.
- **`motor_sequence.py`**: secuencias `ángulo[:dps[:pausa]]` o un CSV `ángulo,velocidad,pausa`.
- **`sysid/`**: `make_sysid_signals.py` genera señales de 1000 puntos a 100 Hz que empiezan y terminan en 0°:
  `chirp` (0.2–15 Hz), `multisine` (24 senoidales, fases de Schroeder), `steps` (±1, 2, 3, 5°) y `ramp` (2 dps).
  Se cargan en `gui.py` con escala 1, ciclos 1, offset 0, 600 dps. `visualizar_sysid.ipynb` las compara con los
  registros reales `ejecucion_*.csv` e **identifica el modelo comando → posición** (2026-11-10):
  - Los `ejecucion_*_0x400.csv` son del modo `MODE_MOTION` (0x400); el resto, del modo posición 0xA4 (más lento).
    `ejecucion_cihrp_0x400.csv` es el chirp (error de tipeo en el nombre).
  - Se usa el modo 0x400. Modelo: **G(s) = 1.013 / (0.0146 s + 1) · e^(−0.005 s)** (ajuste 86–90 %). Lo robusto es
    K ≈ 1 y τ ≈ 15 ms; el retardo (0–5 ms) es poco identificable a 100 Hz, y 2.º/3.er orden no mejoran al 1.º.
  - Resultados en caché: `tf_resultados.json`, `tf_perfil.json` (poner `REFIT = True` en el notebook para
    recalcular) y el resumen exportable `tf_por_modo.json`.
  - Sin modelar: error estático ~0.3° en escalones (zona muerta/fricción).

## Pendientes

Ver la lista completa en [`../docs/historia.md`](../docs/historia.md#pendientes-conocidos). En resumen: calibrar
la transmisión por paralelogramos, corregir la convención "90 = recto" de `plot_leg_actuator_angles.ipynb` y
`phase_match_*.py`, identificar el modelo del MyActuator con los datos de `sysid/` y llevar la estimación de fase
con IMU al hardware real. Del MyActuator ya está el modelo lineal (modo 0x400); falta la zona muerta/fricción,
validarlo con trayectorias reales y diseñar el prefiltro/compensador.

`servos/fr_leg_animation.gif` pesa ~25 MB; si el repo crece, conviene regenerarlo más liviano o excluirlo.
