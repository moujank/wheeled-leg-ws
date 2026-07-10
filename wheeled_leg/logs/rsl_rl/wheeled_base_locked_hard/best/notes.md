# wheeled_base_locked_hard best policy

updated_at: 2026-07-10 13:57:55
source_run: 2026-07-10_11-10-58
source_checkpoint: model_6200.pt
best_model: ../2026-07-10_11-10-58/model_6200.pt
task: Wheeled-Base-Locked-Curriculum
play_task: Wheeled-Base-Locked-Curriculum-Play
status: selected

## Why This Checkpoint

Short reason for selecting this checkpoint.

## Play

```bash
python scripts/rsl_rl/play.py --task=Wheeled-Base-Locked-Curriculum-Play --experiment_name=wheeled_base_locked_hard --load_run=best --checkpoint=model.pt
```

## Train

```bash
python scripts/rsl_rl/train.py --task=Wheeled-Base-Locked-Curriculum --experiment_name=wheeled_base_locked_hard --run_name=<run_name>
```
