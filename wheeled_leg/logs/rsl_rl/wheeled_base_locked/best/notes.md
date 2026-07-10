# wheeled_base_locked best policy

updated_at: 2026-07-09 19:15:45
source_run: 2026-07-09_17-13-00
source_checkpoint: model_3500.pt
best_model: ../2026-07-09_17-13-00/model_3500.pt
task: Wheeled-Base-Locked-simple
play_task: Wheeled-Base-Locked-simple-Play
status: selected

## Why This Checkpoint

Short reason for selecting this checkpoint.

## Play

```bash
python scripts/rsl_rl/play.py --task=Wheeled-Base-Locked-simple-Play --experiment_name=wheeled_base_locked --load_run=best --checkpoint=model.pt
```

## Train

```bash
python scripts/rsl_rl/train.py --task=Wheeled-Base-Locked-simple --experiment_name=wheeled_base_locked --run_name=<run_name>
```
