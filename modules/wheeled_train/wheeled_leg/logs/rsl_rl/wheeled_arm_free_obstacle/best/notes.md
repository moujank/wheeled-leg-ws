# wheeled_arm_free_obstacle best policy

updated_at: TBD
source_run: TBD
source_checkpoint: TBD
best_model: model.pt
task: Wheeled-Arm-Free-Obstacle-v0
play_task: Wheeled-Arm-Free-Obstacle-Play-v0
status: placeholder

## Selection Criteria

Prefer robust obstacle traversal with low fall rate, controlled body attitude, and acceptable forward tracking.

## Play

```bash
python scripts/rsl_rl/play.py --task=Wheeled-Arm-Free-Obstacle-Play-v0 --experiment_name=wheeled_arm_free_obstacle --load_run=best --checkpoint=model.pt
```
