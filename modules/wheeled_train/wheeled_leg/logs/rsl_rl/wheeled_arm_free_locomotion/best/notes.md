# wheeled_arm_free_locomotion best policy

updated_at: 2026-07-03 20:34:37
source_run: 2026-07-03_15-17-52
source_checkpoint: model_15450.pt
best_model: ../2026-07-03_15-17-52/model_15450.pt
task: Wheeled-Arm-Free-Locomotion-v0
play_task: Wheeled-Arm-Free-Locomotion-Play-v0
status: selected

## Why This Checkpoint

Short reason for selecting this checkpoint.

## Play

```bash
python scripts/rsl_rl/play.py --task=Wheeled-Arm-Free-Locomotion-Play-v0 --experiment_name=wheeled_arm_free_locomotion --load_run=best --checkpoint=model.pt
```

## Train

```bash
python scripts/rsl_rl/train.py --task=Wheeled-Arm-Free-Locomotion-v0 --experiment_name=wheeled_arm_free_locomotion --run_name=<run_name>
```
