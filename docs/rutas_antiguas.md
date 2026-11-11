# Rutas antiguas → rutas nuevas

El 2026-11-10 los repositorios `Definicion-Senecabot` y `Senecabot_mini` se fusionaron en este. Los change logs,
los registros de los agentes (`.hydra/`, `wandb/`) y algunos comentarios escritos antes de esa fecha usan las
rutas antiguas. Esta tabla las traduce.

## Definicion-Senecabot → `simulacion/`

Todo lo que estaba en la raíz de `Definicion-Senecabot/` (también llamado "Optimizacion Geometria" en algunos
documentos) está ahora dentro de `simulacion/`. Además:

| Ruta antigua | Ruta nueva |
|---|---|
| `analisis_microsaltos/` | `simulacion/analisis/microsaltos/` |
| `diagnostico_microsaltos/` | `simulacion/analisis/microsaltos_traseros/` |
| `analisis_rr/` | `simulacion/analisis/pata_rr/` |
| `pruebas_pendiente/` | `simulacion/analisis/pendientes/` |
| `my actuator/` | `simulacion/catalogo_myactuator/` |
| `seneca_loco/`, `loco-mujoco-seneca/`, `geometry_opt/`, `requirements.txt` | `simulacion/` + el mismo nombre |
| `README.md`, `CLAUDE.md` | `simulacion/README.md`, `simulacion/CLAUDE.md` |
| `~/Definicion-Senecabot/seneca_loco` (valor de `SENECA_HOME`) | `~/Tesis-Senecabot/simulacion/seneca_loco` |

## Senecabot_mini → `hardware/`

| Ruta antigua | Ruta nueva |
|---|---|
| `eris_servo_ina/`, `eris_servo_wifi/`, `esp32_servo_control/`, `test_ina219/`, `test_bmi160/` | `hardware/firmware/` + el mismo nombre |
| `motor_sequence.py` | `hardware/myactuator_esp32/motor_sequence.py` |
| `servos/requirements.txt` | `hardware/requirements.txt` |
| `servos/`, `leg_transmission/`, `myactuator_esp32/` | `hardware/` + el mismo nombre |
| `README.md`, `CLAUDE.md`, `AGENTS.md` | `hardware/README.md`, `hardware/CLAUDE.md`, `hardware/AGENTS.md` |
| `Tesis/workspace/seneca_loco/...` (proyecto externo citado en las notas) | `simulacion/seneca_loco/...` (ahora en este repo) |

En el commit inicial de `Senecabot_mini` los notebooks de servos estaban en la raíz; se movieron a `servos/` en el
commit siguiente.

## Historial de git

El historial completo de ambos repositorios está incluido. Para ver la historia de un archivo a través de los
movimientos usa `--follow`:

```bash
git log --follow -- simulacion/analisis/pendientes/slope_test.py
```
