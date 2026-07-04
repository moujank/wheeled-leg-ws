# wheeled_mujoco.sim2sim

This folder contains sim2sim runner adapters.

Current implementation:

- `base_locked.py` provides the source-layer API and argparse builder for the existing base-locked MuJoCo rollout.
- It imports the validated implementation from `minimal_sim2sim/run_base_locked.py` instead of copying it, so default parameters remain aligned with the current debug baseline.

Future task-specific runners should be added here only after their observations, actions, actuator order, and policy exports have been checked against the corresponding training task.
