# Entrenamiento

`train.py` entrena con PPO (JAX) usando la configuración Hydra que se le pase. Los agentes quedan en
`$SENECA_HOME/artifacts/trained_agents/` (modelo RRR) o `artifacts/trained_agents_rr/` (modelo RR), en una carpeta
`<fecha>/<hora>` con `PPOJax_final.pkl`, `PPOJax_saved.pkl` (mejor validación), el log y la config de Hydra.

Desde `simulacion/seneca_loco/`, con el `.venv` activado:

| Configuración | Qué hace |
|---|---|
| `conf.yaml` | Modelo RRR desde cero. |
| `conf_finetune_slope.yaml`, `conf_finetune_slope_task.yaml` | Fine-tune RRR en pendientes. |
| `conf_rr.yaml` | Modelo RR desde cero (control por par). |
| `conf_rr_finetune.yaml` | Fine-tune RR (control por par). |
| `conf_rr_finetune_tf.yaml` | Fine-tune del agente RR elegido con **control por posición a través de la función de transferencia del MyActuator** (ver abajo). |

```bash
python simulation/training/train.py --config-name conf_rr_finetune_tf
```

Opciones útiles: sin cuenta de wandb, anteponer `WANDB_MODE=disabled`; con poca VRAM, añadir
`finetune.experiment.num_envs=512 finetune.experiment.env_params.nconmax=6500` (en los fine-tunes los parámetros
van bajo `finetune.experiment`). Evaluar un agente RR: `python simulation/training/eval_rr.py --path <pkl>`.

## Fine-tune con la función de transferencia (`conf_rr_finetune_tf.yaml`)

- La red comanda el **ángulo** de cada manivela. El comando pasa por la G(s) identificada en el motor real (modo
  0x400, `hardware/myactuator_esp32/sysid/`): G(s) = 1.013 / (0.0146 s + 1) · e^(−0.0053 s), y un PD (kp 3200,
  kd 25) saturado al par de cada motor hace que la manivela la siga. Implementación: `TFPDControl` en
  `loco-mujoco-seneca/loco_mujoco/core/control_functions/tf_pd.py`.
- Parte del agente `trained_agents_rr/2026-11-04/17-37-28`, que controlaba por **par**: su capa de salida no sirve
  para ángulos y se pone en cero (`reset_actor_head`), así que se comporta casi como un entrenamiento desde cero.
  Por eso son 100 M pasos (~1.7 h en la GPU con que se entrenó 17-37-28) y pesos de suavidad bajos (0.1).
- Los parámetros de G(s), las ganancias del PD y el rango de cada manivela están en el YAML.
