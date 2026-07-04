# wheeled_grasp_pose_locomotion best policy

updated_at: TBD
source_run: TBD
source_checkpoint: TBD
best_model: model.pt
task: Wheeled-Grasp-Pose-Locomotion-v0
play_task: Wheeled-Grasp-Pose-Locomotion-Play-v0
status: placeholder

## Selection Criteria

Prefer stable locomotion while the arm stays near grasp-ready posture without destabilizing the base.

## Play

```bash
python scripts/rsl_rl/play.py --task=Wheeled-Grasp-Pose-Locomotion-Play-v0 --experiment_name=wheeled_grasp_pose_locomotion --load_run=best --checkpoint=model.pt
```
