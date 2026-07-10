# wheeled_mujoco/source

This folder is the importable source tree for the MuJoCo-side Python code.

## Subpackages

- `wheeled_mujoco/tasks`: task metadata that mirrors the training-side naming and log layout
- `wheeled_mujoco/sim2sim`: runner adapters and CLI-level argument plumbing
- `wheeled_mujoco/ros`: placeholder ROS-facing interfaces
- `wheeled_mujoco/orchestration`: placeholder task switching and coordination helpers
- `wheeled_mujoco/paths.py`: shared repository-relative path helpers

## Design boundary

- reusable Python logic belongs here
- one-off execution entry points belong in `modules/wheeled_mujoco/scripts`
- the low-risk base-locked behavioral reference still lives in `modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py`

At the moment, this package is intentionally small. It wraps the validated baseline instead of replacing it.
