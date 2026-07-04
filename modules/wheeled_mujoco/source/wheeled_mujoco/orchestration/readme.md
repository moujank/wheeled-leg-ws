# wheeled_mujoco.orchestration

This folder reserves task switching and task coupling logic.

Current status:

- `task_switcher.py` provides a small in-process registry wrapper and transition description.
- It does not blend policies or modify MuJoCo control yet.
- Future coupling between locomotion, grasp, obstacle traversal, and recovery should live here before being exposed to ROS nodes.

The boundary is intentionally above individual sim2sim runners. A runner executes one task; the orchestration layer decides which task should be active.
