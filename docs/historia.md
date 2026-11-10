# Historia del proyecto

Qué se hizo, en qué orden y por qué. El detalle técnico de cada cambio está en los change logs
(`simulacion/seneca_loco/CLAUDE.md` → *Change Log*, y `simulacion/loco-mujoco-seneca/SENECA_LOCAL_CHANGES.md`);
este documento es el resumen para orientarse.

---

## Punto de partida (hasta junio de 2026)

**Tesis de Nicolás Andrade Manrique** (Uniandes, junio 2026): modelo MuJoCo de SenecaBot con patas RRR
(cadera, rodilla, tobillo), pipeline MOCAP de una oveja → trayectoria de referencia, y entrenamiento por
imitación estilo DeepMimic con PPO en JAX sobre un fork de LocoMuJoCo. Es lo que hoy está en
`simulacion/seneca_loco/` y `simulacion/loco-mujoco-seneca/`. Su mejor agente caminaba a la velocidad de la oveja
pero con **microsaltos** (hasta 3 toques del pie por ciclo).

## Continuación (desde septiembre de 2026)

El trabajo de Carlos Andrés Tovar avanzó en dos repositorios en paralelo, que aquí se unieron:

- **Simulación** (antes `Definicion-Senecabot`, hoy `simulacion/`).
- **Robot físico** (antes `Senecabot_mini`, hoy `hardware/`).

### Cronología

| Fecha | Parte | Qué se hizo |
|---|---|---|
| 09-24 | hardware | Control de los 12 servos con ESP32 + PCA9685 y medición de corriente con INA219. Se confirma a mano que las patas delanteras y traseras están montadas en espejo (`JOINT_SIGN`). |
| 09-26 | simulación | Arreglo de la caminata scripted (columnas `*_knee_angle`). |
| 09-27 | simulación | **Diagnóstico de los microsaltos**: la referencia avanzaba a 1.40 m/s (velocidad de la oveja), unas 7 veces más de lo que las patas del robot pueden empujar sin deslizar (~0.2 m/s). Se añadieron una velocidad de referencia sin deslizamiento, una recompensa por patrón de contacto y penalizaciones de suavidad. |
| 09-27 | simulación | **Bug de reinicio de MJWarp** arreglado: un entorno con NaN quedaba atascado y hundía la curva de entrenamiento. |
| 09-27 | simulación | Dos agentes de 100 M pasos sin microsaltos: **v0218** (0.21 m/s) y **v140** (1.39 m/s). |
| 09-27 | simulación | **`geometry_opt`**: dinámica inversa sobre la marcha grabada → selección de motores MyActuator → dimensionamiento de eslabones como tubos de PLA/PETG. |
| 09-29 | simulación | **Pendientes**: v140 se cae desde +25° de subida y en −35°. Tras un ajuste fino con recompensa de tarea, camina sin caídas de −35° a +30°. |
| 10-06 | simulación | Los microsaltos traseros que quedaban en v140 vienen de la referencia (apoyo trasero con doble bajada), no del agente. |
| 10-24 | hardware | **Transmisión por paralelogramos**: todos los motores en el torso; se calculan los ángulos de motor del ciclo de marcha. Se verifica que los CSV de marcha son exactamente el `qpos` de la simulación. |
| sin fecha | hardware | Notebooks `Mecanismo*.ipynb`: exploración de una pata con dos mecanismos de barras RR. `MecanismoV2.ipynb` se portó luego a `simulacion/geometry_opt/linkage.py`. |
| 11-03 → 11-04 | simulación | **Rediseño de la pata a RR** (frente activo): sin muslo ni cadera; rodilla y tobillo movidos desde el torso por dos mecanismos de barras. Mecanismo dimensionado, modelo MuJoCo generado, entorno `SenecaBotRR` y agentes entrenados. Elegido: `trained_agents_rr/2026-11-04/17-37-28`. |
| 11-06 | hardware | Seguimiento de trayectoria con el motor MyActuator (registro `track_fr_hip_motor_20261106_*`) y señales de identificación de sistema (`sysid/`) grabadas. |
| 11-10 | — | Los dos repositorios se fusionan en este (`Tesis-Senecabot`), con su historial. |
| 11-10 | simulación | Catálogo MyActuator ampliado desde la web del fabricante: dimensiones de 29 motores, curvas características de 19, 6 motores nuevos y fichas PDF (`simulacion/catalogo_myactuator/README.md`). Cambia la selección de `geometry_opt` (aún no regenerada). |

---

## Resultados principales

### Agentes RRR (100 M pasos)

| Política | Velocidad | Microsaltos | Coincidencia de contacto | Masa final (PLA, `geometry_opt`) |
|---|---|---|---|---|
| Referencia original, recompensa original (tesis) | 1.38 m/s | sí (hasta 3 toques por ciclo) | 61 % | — |
| v0218: referencia sin deslizamiento | 0.21 m/s | no (1 toque por ciclo) | 94 % | 8.77 kg |
| v140: referencia 1.40 m/s + nuevas recompensas | 1.39 m/s | no | 93 % | 9.72 kg |

**Motores y eslabones** (`geometry_opt`): las rodillas delanteras son las más exigentes (**X6-60**); el resto usa
**X4-36 / X8-32 / X4-10**. La velocidad articular, más que el torque, decide la mayoría de los motores. Los
eslabones quedan como tubos de ≈Ø15–19 mm con pared de 4 mm (gobierna la flexión).

### Agente RR elegido (`17-37-28`)

0 caídas en 32 × 1000 pasos, 1.156 m/s (82 % de la referencia), RMS de rodilla/tobillo 9.6°, masa total del
modelo 6.06 kg.

### Dónde están los agentes

| Agente | Ruta (`simulacion/seneca_loco/artifacts/…`) | ¿En el repo? |
|---|---|---|
| Tesis (Nicolás) | `trained_agents/curated/4_best_try/17-19-49/PPOJax_saved.pkl` | no |
| v0218 | `trained_agents/2026-09-27/07-12-14/PPOJax_saved.pkl` | no |
| v140 | `trained_agents/2026-09-27/18-22-14/PPOJax_saved.pkl` | **sí** |
| Pendientes (tarea) | `trained_agents/2026-09-29/18-49-47/PPOJax_final.pkl` | no |
| **RR elegido** | `trained_agents_rr/2026-11-04/17-37-28/PPOJax_final.pkl` | **sí** (junto con los demás agentes RR del 11-03 y 11-04) |

Los marcados "no" se citan en la documentación pero nunca se subieron a git; existen solo en el equipo donde se
entrenaron. Para reproducirlos hay que volver a entrenar (las configuraciones sí están en el repo). Las marchas
grabadas de v0218 y v140 que usa `geometry_opt` sí están (`geometry_opt/results/gait_*.npz`).

---

## Dos diseños de transmisión de la pata

Conviven dos ideas para llevar los motores al torso, y conviene no confundirlas:

| | Paralelogramos | Mecanismo de barras RR |
|---|---|---|
| Dónde | `hardware/leg_transmission/leg_transmission.py` | `simulacion/geometry_opt/linkage.py`, `make_rr_xml.py`, modelo `senecabot_loco_rr.xml` |
| Pata | RRR (conserva la cadera, de accionamiento directo) | RR (se elimina el muslo y la cadera) |
| Motores por pata | 3 | 2 |
| Estado | Ángulos de motor calculados; falta calibración física | Modelo y agentes entrenados en simulación |

La documentación de cada parte no deja escrito cuál de los dos es el definitivo; confírmalo antes de construir,
porque la elección afecta tanto al hardware como al modelo de simulación.

---

## Pendientes conocidos

**Simulación**

- Agente RR: las patas traseras rebotan ~2 cm tras aterrizar; la velocidad queda en el 82 % de la referencia; los
  tobillos delanteros tienen un error RMS alto. Ya se probó sin éxito subir el tronco (agente `18-32-41`).
- `geometry_opt` con el mecanismo no cierra el lazo dinámico: los motores no se mueven al torso ni se suma la masa
  de las barras, y los torques usan la marcha RRR. Se podría grabar la marcha del agente RR y usarla.
- Las patas izquierdas (FL/BL) se copian en espejo de las derechas; no se dimensionan por separado.
- Regenerar `geometry_opt/results_summary.md` con el catálogo MyActuator ampliado (cambia algunos motores) y,
  si sirve, usar las dimensiones de los motores (radio con holgura) en el modelo en vez de masas puntuales.

**Hardware**

- Calibrar offsets y signos físicos de la transmisión por paralelogramos.
- Corregir (o marcar como obsoletos) los cálculos de `plot_leg_actuator_angles.ipynb` y `phase_match_*.py` que
  usan la convención "90 = recto".
- Identificar el modelo comando → posición del MyActuator con los datos de `myactuator_esp32/sysid/`.
- Llevar a hardware real la estimación de fase con un IMU en el tobillo (hoy solo simulada; `firmware/test_bmi160`
  es el primer paso).
- El firmware de la ESP32 para el MyActuator (`motor_control.ino`) no está en este repo; vive en el proyecto
  `PCR_GP-main`.

**General**

- Decidir entre paralelogramos y mecanismo RR (ver arriba).
- Definir una licencia para el código nuevo.
