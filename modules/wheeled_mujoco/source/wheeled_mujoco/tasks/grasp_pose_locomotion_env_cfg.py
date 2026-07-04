"""MuJoCo config for grasp_pose_locomotion."""

from __future__ import annotations

from wheeled_mujoco.tasks.common import CommandRangeCfg, arm_task


GRASP_POSE_LOCOMOTION = arm_task(
    task_name="wheeled_grasp_pose_locomotion",
    train_env_cfg="grasp_pose_locomotion_env_cfg.py:GraspPoseLocomotionEnvCfg_PLAY",
    log_name="wheeled_grasp_pose_locomotion",
    command=CommandRangeCfg(lin_vel_x=(0.4, 0.4), lin_vel_y=(0.0, 0.0), heading=(0.0, 0.0)),
    notes="Couples locomotion and grasp-pose commands. Requires the future orchestration layer.",
)

TASKS = {GRASP_POSE_LOCOMOTION.task_name: GRASP_POSE_LOCOMOTION}
