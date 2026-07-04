# RSL-RL Best Policy Index

This directory keeps one stable `best/` entry for each training stage.

Raw training runs stay in:

```text
logs/rsl_rl/<experiment>/<timestamp_run_name>/
```

The selected policy for play is indexed in:

```text
logs/rsl_rl/<experiment>/best/model.pt
logs/rsl_rl/<experiment>/best/notes.md
```

Use `scripts/rsl_rl/update_best_policy.py` after choosing a checkpoint:

```bash
python scripts/rsl_rl/update_best_policy.py \
  --experiment=wheeled_base_locked \
  --run=<timestamp_run_name> \
  --checkpoint=model_<iteration>.pt \
  --task=Wheeled-Base-Locked-v0 \
  --play-task=Wheeled-Base-Locked-Play-v0 \
  --comment="Short reason for selecting this checkpoint."
```

Then play with:

```bash
python scripts/rsl_rl/play.py \
  --task=<Play-Task-Name> \
  --experiment_name=<experiment> \
  --load_run=best \
  --checkpoint=model.pt
```
