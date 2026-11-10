# Eris — Robot cuadrúpedo: control, transmisión y calibración de patas

Código de tesis para el robot cuadrúpedo **Eris** (modelo MuJoCo `senecabot_loco.xml`). Reúne tres líneas de trabajo:

1. **Servos PWM** (`servos/`): control de las patas con servos vía ESP32 + PCA9685, captura de corriente (INA219) y análisis/animación de trayectorias de marcha.
2. **Transmisión por paralelogramos** (`leg_transmission/`): diseño cinemático para mover rodilla y tobillo desde el torso, y conversión del ciclo de marcha a ángulos de motor.
3. **Actuador MyActuator por CAN** (`myactuator_esp32/`): control de posición de un motor MyActuator a través de una ESP32, seguimiento de trayectorias, GUI e identificación de sistema.

Además hay firmware de ESP32 (`*.ino`) y un sensor IMU de prueba (BMI160).

## Estructura del repositorio

```
.
├── motor_sequence.py            # Secuencia de ángulos al motor MyActuator (CLI)
├── myactuator_esp32/            # Librería, GUI y herramientas del motor MyActuator
│   ├── myactuator_esp32.py      #   Clase MyActuator (pyserial)
│   ├── gui.py                   #   Interfaz gráfica (tkinter + matplotlib)
│   ├── tracking.py              #   Lectura de CSV y comparación enviado vs medido
│   ├── track_csv.py             #   Seguimiento de una columna de CSV (CLI)
│   ├── track_fr_hip_motor_*.csv/png   # Ejemplo de registro de una ejecución
│   └── sysid/                   #   Señales de excitación para identificación
├── leg_transmission/            # Transmisión por paralelogramos (torso -> pata)
│   ├── leg_transmission.py      #   Cálculo de ángulos de motor + figura
│   ├── leg_motor_angles_one_cycle.csv   # Salida: ángulos de motor, 1 ciclo
│   ├── Mecanismo*.ipynb, Version2.ipynb, leg_transmission.ipynb  # Diseño/iteraciones
│   └── *.png, *.gif             #   Figuras y animaciones del mecanismo
├── servos/                      # Servos PWM, marcha y estimación de fase
│   ├── eris_servo_control.ipynb #   Notebook principal (16 canales + INA219)
│   ├── send_servo_command.ipynb #   Banco de pruebas de 3 servos
│   ├── send_serial.ipynb, send_csv_wifi.ipynb   # Envío de CSV por Serial / WiFi
│   ├── plot_leg_actuator_angles.ipynb  # Análisis y animación de la pata FR
│   ├── plot_one_cycle.py        #   Visualizador de un ciclo + suavizado de Fourier
│   ├── phase_match_sim.py       #   Prototipo de estimación de fase con IMU (simulado)
│   ├── phase_match_live.py      #   Versión en vivo del prototipo anterior
│   ├── leg_actuator_angles_*.csv#   Trayectorias articulares (MuJoCo qpos)
│   └── README.md, requirements.txt
├── eris_servo_ina/              # Firmware: 16 servos + INA219 (Serial)
├── eris_servo_wifi/             # Firmware: 16 servos por WiFi (SoftAP + TCP) y Serial
├── esp32_servo_control/         # Firmware: 3 servos con pines directos (Serial)
├── test_ina219/                 # Sketch de prueba del sensor de corriente
├── test_bmi160/                 # Sketch de prueba del IMU BMI160 (I2C directo)
└── tesis/                       # Entorno virtual de Python (no versionar)
```

## Convenciones clave

### Los CSV de marcha usan la convención de MuJoCo
Las columnas `*_joint` (`servos/leg_actuator_angles_*.csv`) son el `qpos` de `senecabot_loco.xml` en grados: **cabeceo absoluto**, no el "90 = recto" de los notebooks de servos. Patas: `fr`, `fl`, `br`, `bl`; articulaciones: `hip`, `knee`, `ankle`. El ciclo base tiene paso de 5 ms (200 Hz).

### Servos: ángulo biomecánico vs. crudo
Las patas delanteras y traseras están armadas en espejo. `move_joint()` y las columnas `<pata>_<eslabón>` usan ángulo biomecánico (0 = reposo, positivo = eslabón hacia arriba, igual sentido en las 4 patas) y se convierten con `JOINT_SIGN`:

| Pata | hip | knee | ankle |
|---|---|---|---|
| FR, FL (delanteras) | +1 | -1 | +1 |
| BR/RR, BL/RL (traseras) | -1 | +1 | -1 |

`raw = 90 + signo · bio`. Para ángulo crudo usar `move_servo(canal, ángulo)` o las columnas `chN`.

### Transmisión por paralelogramos
La cadera es de accionamiento directo; rodilla y tobillo se accionan desde el torso mediante paralelogramos (uno para la rodilla, dos en serie para el tobillo). Un paralelogramo copia orientaciones, así que **cada motor fija el ángulo absoluto de su segmento** respecto al torso:

```
m_hip   = hip                 (muslo)
m_knee  = hip - knee          (tibia)
m_ankle = hip - knee + ankle  (metatarso)
```

La rodilla tiene eje −y en el modelo, por eso entra con signo menos. Longitudes (muslo, tibia, metatarso) en m: delantera `0.121, 0.111, 0.1534`; trasera `0.166, 0.231, 0.148`.

## Módulos

### 1. Servos PWM (`servos/`)
**Hardware:** ESP32 + PCA9685 (16 canales, I2C0: SDA=21, SCL=22) + INA219 (I2C1: SDA=32, SCL=33). 12 servos en uso (hip/knee/ankle × 4 patas).

**Firmware**

| Sketch | Función |
|---|---|
| `eris_servo_ina/` | Firmware principal. 16 servos + corriente INA219, protocolo de texto (`X`, `Y`, `INA`, `S_F`, `S_TIME`, `INFO`, `ON/OFF`). Baud 921600. |
| `eris_servo_wifi/` | Igual sin INA219; la ESP32 crea su propia red (SoftAP, IP `192.168.4.1`, TCP `8888`) y también acepta Serial. |
| `esp32_servo_control/` | 3 servos (pines 13, 26, 15), formato `a1,a2,a3[,duración_ms]`, rampa interpolada. Baud 115200. |
| `test_ina219/` | Verifica que el INA219 responde en el bus. |

**Notebooks y scripts**
- `eris_servo_control.ipynb`: conexión, movimiento por canal/pata/trayectoria CSV, captura de corriente en vivo con exportación a CSV.
- `send_servo_command.ipynb`: pruebas manuales y reproducción de un ciclo en el firmware de 3 servos.
- `send_serial.ipynb` / `send_csv_wifi.ipynb`: reproducción de CSV por Serial o WiFi.
- `plot_leg_actuator_angles.ipynb`: análisis de la trayectoria completa (~95 repeticiones de un ciclo), extracción de un ciclo y animación cinemática de la pata FR.
- `plot_one_cycle.py`: visualiza ángulo/velocidad/aceleración de un ciclo y suaviza con armónicos de Fourier (4, 6, 8, 12) → `leg_actuator_angles_one_cycle_smooth_n4.csv`.
- `phase_match_sim.py` / `phase_match_live.py`: prototipo que estima la **fase del ciclo** de la pata FR a partir de señales de un IMU en el tobillo (ángulo, velocidad, aceleración simulados) y elige hip/knee de la trayectoria predefinida. Compara configuraciones de pesos y modos de búsqueda (`window`, `global`, `recover`) en escenarios limpio/ruidoso/con variación de velocidad.

Nota: el INA219 puede perder su calibración con caídas de tensión; el firmware la rearma cada 2 s.

### 2. Transmisión (`leg_transmission/`)
`leg_transmission.py` lee `servos/leg_actuator_angles_one_cycle.csv`, calcula los ángulos de motor de torso para cada pata (etapas `knee`, `ankle_upper`, `ankle_lower`) y escribe:
- `leg_motor_angles_one_cycle.csv` (columnas `<pata>_<hip|knee|ankle>_motor`)
- `leg_transmission.png`

Los notebooks (`Mecanismo`, `MecanismoV2`, `Version2`, `leg_transmission`) documentan las iteraciones del diseño; los GIF muestran el mecanismo de las patas FR y BR.

### 3. Motor MyActuator por CAN (`myactuator_esp32/`, `motor_sequence.py`)
La ESP32 corre el firmware `motor_control.ino` (**no incluido en este repo**; vive en el proyecto `PCR_GP-main`) que traduce comandos de texto a tramas CAN. No se necesita adaptador USB-CAN, solo `pyserial`.

- **`myactuator_esp32.py`**: clase `MyActuator(port, motor_id)`: `move_to`, `read_angle` (0.01°), `play(tiempos, ángulos)` para trayectorias en tiempo real. La trayectoria (hasta 1000 puntos) se sube a la ESP32 y la reproduce con su propio reloj, leyendo el encoder tras cada comando. Modos: `MODE_POSITION` (0xA4) y `MODE_MOTION` (0x400, lazo kp/kd). Límites: ±360°, 300 dps (comandos), 600 dps (trayectoria), aceleraciones 100–60000 dps².
- **`gui.py`**: GUI con posición manual, gráfica en vivo deseado vs. medido, carga de CSV (tiempo + ángulos), reproducción, pestaña "Última ejecución" y exportación a CSV.
- **`track_csv.py`**: envía una columna de `leg_motor_angles_one_cycle.csv` y reporta RMS, error máximo y retardo.
- **`tracking.py`**: utilidades compartidas (`read_table`, `build_reference`, `analyse`, `save_log`).
- **`sysid/make_sysid_signals.py`**: genera señales de 1000 puntos a 100 Hz (empiezan y terminan en 0°): `chirp` (0.2–15 Hz), `multisine` (24 senoidales, fases de Schroeder), `steps` (±1, 2, 3, 5°) y `ramp` (2 dps). `visualizar_sysid.ipynb` las muestra; los `ejecucion_*.csv` son registros de ejecuciones reales.
- **`motor_sequence.py`** (raíz): secuencias simples desde la línea de comandos.

## Uso rápido

```bash
# Dependencias (servos/requirements.txt): numpy scipy matplotlib sympy pandas jupyter
#   ipywidgets panda3d panda3d-gltf mujoco mediapy pytest prettyplot pyserial
pip install -r servos/requirements.txt

# Regenerar los ángulos de motor desde el ciclo de marcha
python leg_transmission/leg_transmission.py

# Mover el motor a una secuencia de ángulos (ángulo[:dps[:pausa_s]])
python motor_sequence.py --port COM3 0 90 180:60 -45

# Seguir una columna del ciclo y comparar con el encoder
python myactuator_esp32/track_csv.py --port COM3 --column fr_hip_motor

# GUI del motor
python myactuator_esp32/gui.py

# Señales de identificación
python myactuator_esp32/sysid/make_sysid_signals.py --plot

# Prototipo de estimación de fase (desde la raíz, por el import `servos.*`)
python -m servos.phase_match_sim
python -m servos.phase_match_live
```

Para los sketches: Arduino IDE con soporte ESP32 y las librerías `Adafruit PWM Servo Driver Library`, `INA219` (Rob Tillaart) y/o `ESP32Servo` según el sketch. Para los notebooks, ajustar `PORT` al puerto serie propio.

## Estado y notas
- El repositorio se reorganizó en carpetas (`servos/`, `leg_transmission/`, `myactuator_esp32/`) respecto al commit inicial; los archivos de la raíz original aparecen como eliminados en `git status` porque fueron movidos.
- `tesis/` es un entorno virtual local (Python 3.12) y está en `.gitignore`.
- `servos/fr_leg_animation.gif` pesa ~25 MB; considerar Git LFS o excluirlo.
