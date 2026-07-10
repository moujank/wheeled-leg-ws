# wheeled_mujoco/scripts

This folder contains command-line entry points for MuJoCo-side workflows.

## Scripts

- `run_base_locked_sim2sim.py`: direct CLI wrapper for the validated base-locked runner
- `run_task_sim2sim.py`: task dispatcher that resolves a registered MuJoCo task config and forwards arguments to the selected runner

## Current behavior

Today, both scripts eventually route base-locked execution to:

```text
minimal_sim2sim/run_base_locked.py
```

The higher-level `source/wheeled_mujoco/sim2sim/base_locked.py` layer exists so future tasks can share a consistent package boundary without copying the legacy baseline.

## Useful commands

Show the full base-locked CLI:

```bash
python3 modules/wheeled_mujoco/scripts/run_base_locked_sim2sim.py --help
```

Run a short headless base-locked rollout:

```bash
python3 modules/wheeled_mujoco/scripts/run_base_locked_sim2sim.py \
  --no-viewer \
  --no-real-time \
  --duration 5
```

List registered task configs:

```bash
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py --list-tasks
```

Inspect the task runner help:

```bash
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py \
  --task wheeled_base_locked \
  --runner-help
```

Dispatch the same rollout through the task-level wrapper:

```bash
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py \
  --task wheeled_base_locked \
  --no-viewer \
  --no-real-time \
  --duration 5
```

Do not place reusable simulator logic in this directory. Put reusable code under `source/wheeled_mujoco/`.
