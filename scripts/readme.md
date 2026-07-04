# wheeled_mujoco/scripts

This folder contains command-line entry points for MuJoCo sim2sim workflows.

Current scripts:

- `run_base_locked_sim2sim.py`: full argparse command wrapper for the current base-locked policy rollout. It calls the source-layer adapter and preserves the parameters from `minimal_sim2sim/run_base_locked.py`.
- `run_task_sim2sim.py`: task-aware dispatcher. It can list registered task configs and currently routes base-locked tasks to the base-locked runner.

The intended call order is:

```text
scripts/*.py -> source/wheeled_mujoco/sim2sim -> source/wheeled_mujoco/tasks -> asset/log files
```

For the current `wheeled_base_locked` task, the actual runtime path is:

```text
scripts/run_base_locked_sim2sim.py
  -> source/wheeled_mujoco/sim2sim/base_locked.py
  -> minimal_sim2sim/run_base_locked.py
```

This means the script uses the new project structure as the entry point, while still reusing the validated base-locked MuJoCo implementation. The startup, reset, observation, action, and controller behavior remain aligned with the minimal debug baseline.

Run examples:

```bash
# Show the full base-locked CLI.
python3 modules/wheeled_mujoco/scripts/run_base_locked_sim2sim.py --help
```

```bash
# Startup regression through the new script path.
python3 modules/wheeled_mujoco/scripts/run_base_locked_sim2sim.py \
  --no-viewer \
  --no-real-time \
  --no-handoff-log-every-step \
  --duration 4.5 \
  --cmd-vx 0.0
```

```bash
# Short forward run for the validated base-locked task.
python3 modules/wheeled_mujoco/scripts/run_base_locked_sim2sim.py \
  --no-viewer \
  --no-real-time \
  --no-handoff-log-every-step \
  --duration 6 \
  --cmd-vx 0.1
```

```bash
# Task-dispatch entry point for the same base-locked rollout.
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py \
  --task wheeled_base_locked \
  --no-viewer \
  --no-real-time \
  --no-handoff-log-every-step \
  --duration 4.5 \
  --cmd-vx 0.0
```

```bash
# Launch the task-dispatch entry point with the MuJoCo viewer enabled.
# Viewer is enabled by default, so `--viewer` is optional here.
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py \
  --task wheeled_base_locked \
  --viewer \
  --real-time \
  --duration 30 \
  --cmd-vx 0.5
```

```bash
# Show the full forwarded runner CLI for the selected task.
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py \
  --task wheeled_base_locked \
  --runner-help
```

```bash
# Inspect the registered MuJoCo task list.
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py --list-tasks
```

Verification note:

```text
The new script path was checked against the existing startup regression.
Running `scripts/run_base_locked_sim2sim.py` reaches the same 4.5 s startup state as the minimal runner:
base_pos=(+0.007, -0.002, +0.324), rpy_deg=(-0.4, -2.3, -2.8), base_contact=False, leg_contact=False.
```

Do not place reusable simulation logic here. Put reusable logic under `source/wheeled_mujoco`.
