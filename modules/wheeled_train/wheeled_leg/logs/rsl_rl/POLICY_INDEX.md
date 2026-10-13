# RSL-RL Best Policy Index

[Training project](../../README.md) · [中文工程说明](../../README.zh.md)

## Experiment notes

Each row links to the experiment's own checkpoint source, task names, selection status, and rationale. Update those records with the policy helper; the table keeps navigation in one place.

| Experiment | Selection record |
| --- | --- |
| wheeled_base_locked | [notes](wheeled_base_locked/best/notes.md) |
| wheeled_base_locked_simple | [notes](wheeled_base_locked_simple/best/notes.md) |
| wheeled_base_locked_hard | [notes](wheeled_base_locked_hard/best/notes.md) |
| wheeled_arm_free_locomotion | [notes](wheeled_arm_free_locomotion/best/notes.md) |
| wheeled_arm_free_obstacle | [notes](wheeled_arm_free_obstacle/best/notes.md) |
| wheeled_arm_free_recovery | [notes](wheeled_arm_free_recovery/best/notes.md) |
| wheeled_cartesian_grasp | [notes](wheeled_cartesian_grasp/best/notes.md) |
| wheeled_grasp_pose_locomotion | [notes](wheeled_grasp_pose_locomotion/best/notes.md) |
| wheeled_loaded_grasp_obstacle | [notes](wheeled_loaded_grasp_obstacle/best/notes.md) |

## Policy selection workflow

Run the commands below from `modules/wheeled_train/wheeled_leg`. Confirm the task in the project README or registry before selecting a checkpoint.

This directory keeps one stable `best/` entry for each training stage.

Raw training runs stay in:

```text
logs/rsl_rl/<experiment>/<timestamp_run_name>/
```

The selected policy for play is indexed in:

```text
logs/rsl_rl/<experiment>/best/model.pt
logs/rsl_rl/<experiment>/best/notes.md
```

Use `scripts/rsl_rl/update_best_policy.py` after choosing a checkpoint:

```bash
python scripts/rsl_rl/update_best_policy.py \
  --experiment=wheeled_base_locked \
  --run=<timestamp_run_name> \
  --checkpoint=model_<iteration>.pt \
  --task=Wheeled-Base-Locked-simple \
  --play-task=Wheeled-Base-Locked-simple-Play \
  --comment="Short reason for selecting this checkpoint."
```

Then play with:

```bash
python scripts/rsl_rl/play.py \
  --task=<Play-Task-Name> \
  --experiment_name=<experiment> \
  --load_run=best \
  --checkpoint=model.pt
```
