# wheeled_arm_free_recovery best policy

updated_at: TBD
source_run: TBD
source_checkpoint: TBD
best_model: model.pt
task: Wheeled-Arm-Free-Recovery-v0
play_task: Wheeled-Arm-Free-Recovery-Play-v0
status: placeholder

## Selection Criteria

Prefer fast recovery from disturbed or fallen initial states, stable final posture, and low action jitter.

## Play

```bash
python scripts/rsl_rl/play.py --task=Wheeled-Arm-Free-Recovery-Play-v0 --experiment_name=wheeled_arm_free_recovery --load_run=best --checkpoint=model.pt
```
