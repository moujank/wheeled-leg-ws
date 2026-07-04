# wheeled_base_locked_hard best policy

updated_at: 2026-07-03 15:15:22
source_run: 2026-07-03_13-45-06
source_checkpoint: model_4200.pt
best_model: ../2026-07-03_13-45-06/model_4200.pt
task: Wheeled-Base-Hard-Locked-v0
play_task: Wheeled-Base-Locked-Hard-Play-v0
status: selected

## Why This Checkpoint

Short reason for selecting this checkpoint.

## Play

```bash
python scripts/rsl_rl/play.py --task=Wheeled-Base-Locked-Hard-Play-v0 --experiment_name=wheeled_base_locked_hard --load_run=best --checkpoint=model.pt
```

## Train

```bash
python scripts/rsl_rl/train.py --task=Wheeled-Base-Hard-Locked-v0 --experiment_name=wheeled_base_locked_hard --run_name=<run_name>
```
