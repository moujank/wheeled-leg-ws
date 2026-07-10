# wheeled_mujoco source package

`wheeled_mujoco` is the Python package boundary for MuJoCo-side code in this workspace.

## Responsibilities

- mirror training task names with MuJoCo task configs
- resolve repository-relative asset and policy paths
- expose runner adapters that scripts can call
- reserve interfaces for future ROS integration and multi-task orchestration

## Important constraints

- the package is intentionally independent from Isaac Lab runtime imports
- the validated base-locked rollout still comes from `minimal_sim2sim/run_base_locked.py`
- only the base-locked path is runnable today; the other task configs mainly preserve naming and asset/policy mapping

## Import flow

```text
scripts/*.py
  -> wheeled_mujoco.sim2sim
  -> wheeled_mujoco.tasks
  -> wheeled_mujoco.paths
```
