# wheeled_mujoco.sim2sim

This folder contains the runner adapter layer for MuJoCo sim-to-sim execution.

## Current files

- `base_locked.py`: parser builder and execution adapter for the base-locked rollout
- `__init__.py`: public exports for script-side imports

## Current design

`base_locked.py` does not duplicate the full rollout implementation. Instead, it imports and forwards to the validated baseline in:

```text
modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py
```

That keeps the behavioral reference in one place while still letting higher-level scripts and task dispatchers use a package-level API.

Future task-specific runners should be added here only after their observation definitions, action ordering, asset wiring, and exported policies are checked against training.
