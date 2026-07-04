# wheeled_mujoco.ros

This folder reserves ROS-facing interfaces for future MuJoCo simulation control.

Current status:

- No ROS runtime dependency is imported here.
- `interfaces.py` defines small protocols and node specs for future command, state, policy, and task-control nodes.
- Actual ROS2 packages, launch files, and message bindings should be added later without changing the sim2sim core API.

Expected future node roles:

- Command input node: receives body-frame velocity or task commands.
- MuJoCo state node: publishes robot state from simulation.
- Policy runner node: converts state and commands into actions.
- Task supervisor node: switches or blends locomotion, recovery, grasp, and obstacle tasks.
