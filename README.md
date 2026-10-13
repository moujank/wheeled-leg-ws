# wheeled_mujoco

[中文](README.zh.md) · [Module documentation](docs/README.md)

This module contains the MuJoCo-side workspace for the wheeled-leg project.

## Structure

```text
modules/wheeled_mujoco/
├── asset/                 # MJCF, URDF mirrors, mesh assets, and index helpers
├── minimal_sim2sim/       # Validated minimal base-locked MuJoCo rollout
├── scripts/               # Command-line entry points
├── source/                # Importable Python package
└── docs/                  # Architecture, consistency, handoff, asset index, history
```

## Current scope

- provide MuJoCo robot assets for `wheeled_leg` and `wheeled_arm`
- run exported policies from `modules/wheeled_train/wheeled_leg/logs/rsl_rl`
- keep one validated, low-risk base-locked rollout path
- reserve task metadata and script boundaries for future MuJoCo task coverage

## Current runtime status

- `wheeled_base_locked` is the only fully runnable task path today
- `wheeled_base_locked_hard` reuses the same runner but should still be treated as a validation target, not a locked baseline
- arm, grasp, obstacle, and recovery task names are registered only as reserved MuJoCo configs

## Call flow

```text
scripts/run_task_sim2sim.py
  -> source/wheeled_mujoco/sim2sim
  -> source/wheeled_mujoco/tasks
  -> minimal_sim2sim/run_base_locked.py
  -> asset/ + wheeled_train exported policies
```

## Typical commands

List the registered MuJoCo task configs:

```bash
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py --list-tasks
```

Run the validated base-locked path through the script wrapper:

```bash
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py \
  --task wheeled_base_locked \
  --no-viewer \
  --no-real-time \
  --duration 5
```

Run the minimal baseline directly:

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --duration 5
```

Use the more specific README files in `minimal_sim2sim/`, `scripts/`, and `source/` for details.

## Documentation navigation

[Minimal rollout](minimal_sim2sim/README.md) · [CLI entry points](scripts/README.md) · [Source package](source/wheeled_mujoco/README.md) · [Technical notes](docs/README.md)

Historical validation results apply to their recorded policies and parameters. Consult the [debug handoff](docs/MUJOCO_DEBUG_HANDOFF.md) and selected checkpoint for policy status. Run the commands above from the workspace root.
