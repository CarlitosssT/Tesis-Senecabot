# CLAUDE.md — Tesis-Senecabot (raíz)

Mapa para Claude Code (y personas) que retoman el proyecto. Usuario: Carlos Andrés Tovar (Uniandes). **Habla en
español**: responder en español, directo y conciso. Rama `main`; commit/push solo si el usuario lo pide.

## Qué hay aquí

Repo unificado (2026-11-10) de dos repos anteriores, con su historial:

| Carpeta | Antes | Contexto detallado |
|---|---|---|
| `simulacion/` | `Definicion-Senecabot` | `simulacion/CLAUDE.md` (mapa, agentes, pata RR, lecciones) y `simulacion/seneca_loco/CLAUDE.md` (arquitectura + Change Log) |
| `hardware/` | `Senecabot_mini` | `hardware/CLAUDE.md` (servos, transmisión, MyActuator, preferencias del usuario) |
| `docs/` | — | `historia.md`, `convenciones.md`, `glosario.md`, `rutas_antiguas.md` |

**Lee el `CLAUDE.md` de la carpeta en la que vas a trabajar antes de cambiar nada.** Las convenciones de ángulos
(`docs/convenciones.md`) son la fuente de errores más común.

## Reglas rápidas

- Las dos partes tienen entornos distintos: `simulacion/` corre en Linux/WSL2 con GPU (`.venv` en la raíz del repo,
  comandos desde `simulacion/`, `SENECA_HOME=<repo>/simulacion/seneca_loco`); `hardware/` corre en Windows con
  `pyserial` (comandos desde `hardware/`).
- `geometry_opt/` debe seguir siendo hermano de `seneca_loco/` dentro de `simulacion/` (lo importan por ruta relativa).
- Rutas antiguas en change logs y registros de agentes (`Definicion-Senecabot/...`, `analisis_rr/`, `my actuator/`, …):
  tradúcelas con `docs/rutas_antiguas.md`; no reescribas los change logs históricos.
- Documentar cambios según `docs/convenciones.md` §4 (`[SENECA LOCAL CHANGE]`, change logs con fecha, más nuevo arriba).
- Al cambiar algo que afecte el panorama general, actualiza `docs/historia.md` (cronología y pendientes).
- No añadir `__pycache__/` ni entornos virtuales; GitHub rechaza archivos de más de 100 MB.
