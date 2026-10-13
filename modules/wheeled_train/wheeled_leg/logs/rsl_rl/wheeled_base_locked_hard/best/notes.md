# wheeled_base_locked_hard best policy

updated_at: 2026-08-08 13:08:35
source_run: 2026-08-07_21-37-26
source_checkpoint: model_6900.pt
best_model: ../2026-08-07_21-37-26/model_6900.pt
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
