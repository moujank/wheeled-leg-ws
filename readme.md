# wheeled_mujoco

This module contains MuJoCo assets, sim2sim runners, and future ROS/task orchestration scaffolding for the wheeled-leg workspace.

Current layout:

- `asset/`: MuJoCo MJCF, URDF, mesh, texture, and related robot assets.
- `minimal_sim2sim/`: current validated debug baseline. Keep this stable unless explicitly debugging the baseline.
- `scripts/`: command-line entry points for running or inspecting sim2sim tasks.
- `source/`: importable MuJoCo-side package with task configs, runner adapters, ROS interfaces, and task orchestration placeholders.
- `MUJOCO_DEBUG_HANDOFF.md`: state file for the current MuJoCo sim2sim debugging status.

Call order for new sim2sim work:

```text
scripts/run_*.py
  -> source/wheeled_mujoco/sim2sim
  -> source/wheeled_mujoco/tasks
  -> asset/ and wheeled_train logs
```

The existing `minimal_sim2sim/run_base_locked.py` remains the behavior reference for base-locked startup and reset stability.
