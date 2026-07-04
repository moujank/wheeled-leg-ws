# wheeled_loaded_grasp_obstacle best policy

updated_at: TBD
source_run: TBD
source_checkpoint: TBD
best_model: model.pt
task: Wheeled-Loaded-Grasp-Obstacle-v0
play_task: Wheeled-Loaded-Grasp-Obstacle-Play-v0
status: placeholder

## Selection Criteria

Prefer stable loaded/grasp-like locomotion and obstacle traversal with low undesired contact and low arm swing.

## Play

```bash
python scripts/rsl_rl/play.py --task=Wheeled-Loaded-Grasp-Obstacle-Play-v0 --experiment_name=wheeled_loaded_grasp_obstacle --load_run=best --checkpoint=model.pt
```
