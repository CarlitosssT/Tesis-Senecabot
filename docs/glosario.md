# Glosario

Términos y nombres que aparecen en el código y en la documentación.

## El robot

| Término | Significado |
|---|---|
| **SenecaBot** | El robot cuadrúpedo bioinspirado en la cabra. Nombre del proyecto. |
| **Eris** | Nombre con el que aparece el robot en el código de `hardware/` (firmware `eris_servo_*`, notebook `eris_servo_control.ipynb`). |
| **RRR** | Pata original de tres articulaciones de revolución: cadera (*hip*), rodilla (*knee*) y tobillo (*ankle*). Modelo `senecabot_loco.xml`. |
| **RR** | (1) Mecanismo plano de dos barras y dos articulaciones de revolución. (2) La **pata rediseñada** sin cadera, cuya tibia y metatarso se mueven con dos mecanismos RR desde el torso (`senecabot_loco_rr.xml`, entorno `SenecaBotRR`). (3) En `hardware/servos/`, a veces, la pata trasera derecha (*rear right*). Ver [`convenciones.md`](convenciones.md#1-nombres-de-patas-y-articulaciones). |
| **`rr1`, `rr2`** | Manivelas del mecanismo RR: `rr1` mueve la tibia (barras `l1`, `l2`), `rr2` el metatarso (barras `l3`, `l4`). |
| **`f1`, `f2`** | Fracción de la tibia / metatarso donde se conecta el mecanismo RR. |
| **Muslo, tibia, metatarso** | Los tres segmentos de la pata RRR (de arriba abajo). |
| **Paralelogramos** | Transmisión del robot físico que lleva los motores de rodilla y tobillo al torso conservando la cadera (`hardware/leg_transmission/`). |

## Simulación y aprendizaje

| Término | Significado |
|---|---|
| **MuJoCo / MJX / MJWarp** | Simulador físico y sus versiones aceleradas en GPU (JAX y NVIDIA Warp). |
| **LocoMuJoCo** | Librería de aprendizaje por imitación para locomoción. Aquí se usa un fork: `loco-mujoco-seneca`. |
| **MOCAP / C3D** | Captura de movimiento de la oveja; `.c3d` es el formato de los datos crudos. |
| **Referencia** | Trayectoria articular objetivo que el agente imita (`data/processed_data/trajectory_adapted*.npz`). |
| **DeepMimic** | Método de imitación: el agente recibe recompensa por parecerse a la referencia. |
| **PPO** | *Proximal Policy Optimization*, el algoritmo de RL usado (`PPOJax`). |
| **Agente / política** | La red entrenada, guardada como `PPOJax_saved.pkl` (mejor en validación) o `PPOJax_final.pkl` (última). |
| **v0218, v140** | Los dos agentes RRR de 100 M pasos, por la velocidad de su referencia: 0.218 m/s y 1.40 m/s. |
| **Microsaltos** (*micro-hops*) | Varios toques cortos del pie por ciclo en vez de un apoyo limpio. Problema principal que se corrigió. |
| **Velocidad sin deslizamiento** (*no-slip*) | La velocidad a la que el tronco puede avanzar con las patas del robot sin que los pies deslicen (~0.218 m/s). |
| **Coincidencia de contacto** | % del tiempo en que el patrón de apoyo de cada pata coincide con el de la referencia. |
| **Hydra** | Sistema de configuración; los `.yaml` de `simulation/training/` y la carpeta `.hydra/` de cada agente. |
| **wandb** | Weights & Biases, registro de los entrenamientos. Se desactiva con `WANDB_MODE=disabled`. |
| **Fine-tune** | Ajuste fino: seguir entrenando un agente existente con otra configuración (p. ej. pendientes). |
| **`[SENECA LOCAL CHANGE]`** | Marca en el código del fork para cada cambio hecho sobre LocoMuJoCo. |

## Hardware

| Término | Significado |
|---|---|
| **ESP32** | Microcontrolador que maneja servos, sensores y el bus CAN. Se programa con Arduino IDE. |
| **PCA9685** | Controlador PWM de 16 canales para los servos (I2C). |
| **INA219** | Sensor de corriente / voltaje (I2C, en un bus aparte). |
| **BMI160** | IMU (acelerómetro + giroscopio) de prueba para estimar la fase de la marcha. |
| **MyActuator** | Fabricante de los actuadores (series L, H y X, p. ej. X6-60) que se seleccionan en `geometry_opt` y del motor que se prueba por CAN. |
| **CAN** | Bus de comunicación del motor MyActuator. La ESP32 traduce comandos de texto a tramas CAN. |
| **dps** | Grados por segundo (*degrees per second*). |
| **Ángulo biomecánico / crudo** | Convenciones de ángulo de los servos; ver [`convenciones.md`](convenciones.md#b-convención-de-los-servos-físicos-hardwareservos). |
| **sysid** | Identificación de sistema: señales de excitación (chirp, multiseno, escalones, rampa) para modelar la respuesta del motor. |
| **Estimación de fase** | Prototipo que, a partir de un IMU en el tobillo, estima en qué punto del ciclo de marcha está la pata. |
