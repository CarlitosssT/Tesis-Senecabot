# Análisis

Estudios puntuales hechos sobre los agentes entrenados. Cada carpeta responde una pregunta concreta y guarda
sus scripts junto con los resultados (figuras, JSON, videos) que produjeron.

| Carpeta | Pregunta | Resultado |
|---|---|---|
| [`microsaltos/`](microsaltos/) | ¿El agente da microsaltos? ¿Desliza? ¿Qué tan bien imita? (antes y después de corregir la referencia y la recompensa) | Figuras `altura_patas_*.png`. Los agentes nuevos dan un toque por ciclo. |
| [`microsaltos_traseros/`](microsaltos_traseros/) | ¿De dónde salen los microsaltos que quedan en las patas traseras de v140? | `hop_diagnosis_v140.json`: vienen de la referencia (apoyo trasero con doble bajada), no del agente. |
| [`pendientes/`](pendientes/) | ¿Hasta qué pendiente sube y baja el robot sin caerse? ¿Por qué se cae? | v140 cae desde +25° y en −35°; con ajuste fino, sin caídas de −35° a +30°. JSON de resultados y videos en `videos/`. |
| [`pata_rr/`](pata_rr/) | ¿Cómo camina el agente con la pata RR? | `patas_rr.ipynb`: gráficas, GIF de las patas, figura de la geometría 2D; rollouts `.npz` de varios agentes RR. |

## Cómo se ejecutan

Todos necesitan el entorno de simulación instalado (ver [`../README.md`](../README.md)).

**`microsaltos/`** — se ejecutan desde `simulacion/seneca_loco/` (importan `simulation.analysis.*`):

```bash
cd simulacion/seneca_loco
python ../analisis/microsaltos/hops_compare.py AGENTE.pkl REFERENCIA.npz ETIQUETA FIGURA.png
python ../analisis/microsaltos/precision.py    AGENTE.pkl REFERENCIA.npz ETIQUETA
python ../analisis/microsaltos/slip.py         AGENTE.pkl REFERENCIA.npz ETIQUETA
```

**`pendientes/` y `microsaltos_traseros/`** — se ejecutan desde `simulacion/` (importan `geometry_opt`):

```bash
cd simulacion
PYTHONPATH=. python analisis/pendientes/slope_test.py --agent v140 --angles 0 5 10 15 20
PYTHONPATH=. python analisis/pendientes/gait_by_slope.py --agent v140
PYTHONPATH=. MUJOCO_GL=egl python analisis/pendientes/render_slopes.py --agent v140
python analisis/microsaltos_traseros/hop_diagnosis.py --agent v140
```

El docstring al inicio de cada script explica sus opciones y qué archivo escribe.

**`pata_rr/patas_rr.ipynb`** — ábrelo con `analisis/pata_rr/` como carpeta de trabajo (calcula las rutas desde ahí).

> Algunos scripts usan agentes que no están en el repo (v0218 en `slope_diagnosis.py`, el agente de pendientes
> `2026-09-29/18-49-47`); ver la tabla de agentes en [`docs/historia.md`](../../docs/historia.md#dónde-están-los-agentes).
