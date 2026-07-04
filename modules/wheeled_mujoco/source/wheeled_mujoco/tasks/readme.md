# wheeled_mujoco.tasks

This folder mirrors the Isaac training task names with MuJoCo-side asset and environment configuration.

Current scope:

- Register task metadata, asset paths, default exported policy paths, and command ranges.
- Keep `base_locked_locomotion` runnable through the validated base-locked sim2sim adapter.
- Reserve the arm/grasp/obstacle/recovery tasks with explicit MuJoCo asset and log mapping, without pretending their full observations/actions are implemented yet.

Naming follows the training folder:

```text
wheeled_train/.../wheeled_leg/*_env_cfg.py
```

The actual runner dispatch happens in `wheeled_mujoco.sim2sim`.
