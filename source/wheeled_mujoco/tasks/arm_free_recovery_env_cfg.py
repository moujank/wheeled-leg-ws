"""MuJoCo config for arm_free_recovery."""

from __future__ import annotations

from wheeled_mujoco.tasks.common import CommandRangeCfg, arm_task


ARM_FREE_RECOVERY = arm_task(
    task_name="wheeled_arm_free_recovery",
    train_env_cfg="arm_free_recovery_env_cfg.py:ArmFreeRecoveryEnvCfg_PLAY",
    log_name="wheeled_arm_free_recovery",
    command=CommandRangeCfg(lin_vel_x=(0.0, 0.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.0, 0.0), heading=None),
    notes="Recovery task needs explicit reset-pose and recovery-state interfaces before rollout.",
)

TASKS = {ARM_FREE_RECOVERY.task_name: ARM_FREE_RECOVERY}
