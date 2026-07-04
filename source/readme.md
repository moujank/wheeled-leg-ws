# wheeled_mujoco/source

This folder is the importable source workspace for MuJoCo-side sim2sim code.

Responsibilities:

- `wheeled_mujoco/sim2sim`: runnable sim2sim adapters and CLIs.
- `wheeled_mujoco/tasks`: task-level MuJoCo asset/environment configuration, named after the training task files.
- `wheeled_mujoco/ros`: reserved ROS node/interface definitions for future robot-control simulation.
- `wheeled_mujoco/orchestration`: reserved task switching and coupled-task execution utilities.

Existing files outside this source tree are kept as-is. The current stable `minimal_sim2sim/run_base_locked.py` remains the behavioral baseline.
