# wheeled_cartesian_grasp best policy

updated_at: TBD
source_run: TBD
source_checkpoint: TBD
best_model: model.pt
task: Wheeled-Cartesian-Grasp-v0
play_task: Wheeled-Cartesian-Grasp-Play-v0
status: placeholder

## Selection Criteria

Prefer low end-effector position error, stable base posture, and smooth arm motion.

## Play

```bash
python scripts/rsl_rl/play.py --task=Wheeled-Cartesian-Grasp-Play-v0 --experiment_name=wheeled_cartesian_grasp --load_run=best --checkpoint=model.pt
```
