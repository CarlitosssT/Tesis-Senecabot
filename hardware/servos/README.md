# Eris — Control y calibración de patas de robot cuadrúpedo

Código de tesis para controlar, calibrar y analizar las patas de un robot cuadrúpedo (proyecto **Eris**) con un ESP32, servos PWM (PCA9685) y un sensor de corriente INA219. Incluye notebooks de Jupyter para mandar comandos por Serial, capturar corriente por servo, y analizar/animar trayectorias de ángulos articulares, más los sketches de Arduino/ESP32 que corren en la placa.

## Hardware

- **ESP32** como controlador, hablando por Serial USB con la PC.
- **PCA9685** (16 canales PWM) para los servos, en el bus I2C0 (`Wire`, SDA=21, SCL=22).
- **INA219** para medir corriente/voltaje/potencia por servo, en un bus I2C1 aparte (`Wire1`, SDA=32, SCL=33) para no interferir con el PCA9685.
- 12 servos usados (hip/knee/ankle × 4 patas: FR, FL, RR, RL); ver mapeo de canales en `LEG_JOINT_CHANNELS` dentro de `eris_servo_control.ipynb`.

## Estructura del repo

### Firmware (Arduino/ESP32)

| Sketch | Para qué |
|---|---|
| `eris_servo_ina/eris_servo_ina.ino` | Firmware principal: 16 servos vía PCA9685 + lectura de corriente INA219, protocolo de texto compatible con el flavor `ErisServoDriver`. Usado por `eris_servo_control.ipynb`. |
| `esp32_servo_control/esp32_servo_control.ino` | Firmware simple de 3 servos (una sola pata, pines directos del ESP32), usado por `send_servo_command.ipynb` para pruebas puntuales. |
| `test_ina219/test_ina219.ino` | Sketch mínimo para confirmar que el INA219 responde en el bus I2C (útil si aparece "INA not found"). |

### Notebooks (Python)

| Notebook | Para qué |
|---|---|
| `eris_servo_control.ipynb` | Notebook principal: conexión Serial, movimiento de los 16 canales (crudo, por pata/eslabón o por trayectoria CSV), ventana emergente de captura de corriente INA219 con gráfico en vivo y exportación a CSV. |
| `send_servo_command.ipynb` | Banco de pruebas para el firmware de 3 servos (`esp32_servo_control.ino`): comandos manuales, sliders, y reproducción de `leg_actuator_angles_one_cycle.csv`. |
| `plot_leg_actuator_angles.ipynb` | Analiza `leg_actuator_angles_trajectory_adapted.csv` (trayectoria planeada de las 4 patas): grafica los ángulos articulares, extrae un ciclo único, y reconstruye/anima la cinemática de la pata FR. |

### Datos

- `leg_actuator_angles_trajectory_adapted.csv` — trayectoria completa planeada (todas las patas, ~95 repeticiones de un mismo ciclo).
- `leg_actuator_angles_one_cycle.csv` — un solo ciclo extraído del CSV anterior (ver `plot_leg_actuator_angles.ipynb`).
- `fr_leg_initial_pose.png`, `fr_leg_animation.gif` — salidas visuales generadas por `plot_leg_actuator_angles.ipynb`.

## Convención de ángulos (importante)

En `eris_servo_control.ipynb`, `move_joint()` y las trayectorias CSV con columnas `<pata>_<eslabón>` (ej. `FR_hip`) usan **ángulo biomecánico**: `0` = reposo (servo horizontal, ángulo crudo 90°), positivo = eslabón hacia arriba, **mismo sentido en las 4 patas**.

Internamente esto se convierte al ángulo crudo del servo con `JOINT_SIGN`, porque las patas traseras están armadas espejadas respecto a las delanteras:

| Pata | hip | knee | ankle |
|---|---|---|---|
| FR, FL (delanteras) | +1 | -1 | +1 |
| RR, RL (traseras) | -1 | +1 | -1 |

`raw = 90 + signo · bio`. Para mandar el ángulo crudo del servo tal cual (sin esta corrección), usar `move_servo(channel, angle)` o las columnas `chN` del CSV de trayectoria.

## Setup

1. Python 3.10+ y las dependencias:

   ```bash
   pip install -r requirements.txt
   ```

2. Arduino IDE con **ESP32 board support**, y las librerías: `Adafruit PWM Servo Driver Library`, `INA219` (Rob Tillaart) o `Adafruit INA219` según el sketch.
3. Subir el sketch correspondiente al ESP32 (`eris_servo_ina.ino` para el notebook principal, `esp32_servo_control.ino` para el banco de pruebas de 3 servos).
4. Abrir el notebook deseado en Jupyter, ajustar `PORT` a tu puerto serie, y ejecutar las celdas en orden.

## Notas

- La captura de corriente depende de que el INA219 esté bien alimentado; caídas de tensión fuertes (sag) pueden resetear su calibración interna y dejar la corriente leída en 0 aunque el voltaje se siga midiendo bien — el firmware re-arma la calibración cada 2 s como salvaguarda.
- La carpeta `tesis/` es un entorno virtual de Python local y no debería versionarse (ver `.gitignore`).
