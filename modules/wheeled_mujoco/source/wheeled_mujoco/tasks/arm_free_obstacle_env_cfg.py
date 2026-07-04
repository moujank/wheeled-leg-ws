"""MuJoCo config for arm_free_obstacle."""

from __future__ import annotations

from wheeled_mujoco.tasks.common import CommandRangeCfg, arm_task


ARM_FREE_OBSTACLE = arm_task(
    task_name="wheeled_arm_free_obstacle",
    train_env_cfg="arm_free_obstacle_env_cfg.py:ArmFreeObstacleEnvCfg_PLAY",
    log_name="wheeled_arm_free_obstacle",
    command=CommandRangeCfg(lin_vel_x=(0.8, 0.8), lin_vel_y=(0.0, 0.0), heading=(0.0, 0.0)),
    notes="Obstacle terrain and contact setup are reserved for a task-specific MuJoCo scene builder.",
)

TASKS = {ARM_FREE_OBSTACLE.task_name: ARM_FREE_OBSTACLE}
