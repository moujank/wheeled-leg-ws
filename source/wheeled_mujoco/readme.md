# wheeled_mujoco source package

This package holds MuJoCo-side task configuration, sim2sim runners, ROS-facing interfaces, and future task orchestration code.

The package is intentionally independent from Isaac Lab. It mirrors the task names and log-policy layout from `modules/wheeled_train`, while using MuJoCo assets from `modules/wheeled_mujoco/asset`.

Import order:

```text
scripts
  -> wheeled_mujoco.sim2sim
  -> wheeled_mujoco.tasks
  -> wheeled_mujoco.paths
```

ROS and task switching code are currently interface scaffolds. They should not be used to alter the validated base-locked startup behavior until their runtime nodes are implemented and tested.
