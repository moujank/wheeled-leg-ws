# wheeled_base_locked best policy

updated_at: 2026-07-03 12:41:09
source_run: 2026-07-03_09-52-21
source_checkpoint: model_10600.pt
best_model: ../2026-07-03_09-52-21/model_10600.pt
task: Wheeled-Base-Locked-v0
play_task: Wheeled-Base-Locked-Play-v0
status: selected

## Why This Checkpoint

Short reason for selecting this checkpoint.

## Play

```bash
python scripts/rsl_rl/play.py --task=Wheeled-Base-Locked-Play-v0 --experiment_name=wheeled_base_locked --load_run=best --checkpoint=model.pt
```

## Train

```bash
python scripts/rsl_rl/train.py --task=Wheeled-Base-Locked-v0 --experiment_name=wheeled_base_locked --run_name=<run_name>
```
