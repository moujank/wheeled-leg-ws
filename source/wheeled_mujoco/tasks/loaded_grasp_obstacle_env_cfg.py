"""MuJoCo config for loaded_grasp_obstacle."""

from __future__ import annotations

from wheeled_mujoco.tasks.common import CommandRangeCfg, arm_task


LOADED_GRASP_OBSTACLE = arm_task(
    task_name="wheeled_loaded_grasp_obstacle",
    train_env_cfg="loaded_grasp_obstacle_env_cfg.py:LoadedGraspObstacleEnvCfg_PLAY",
    log_name="wheeled_loaded_grasp_obstacle",
    command=CommandRangeCfg(lin_vel_x=(0.5, 0.5), lin_vel_y=(0.0, 0.0), heading=(0.0, 0.0)),
    notes="Loaded grasp plus obstacle task. Needs object/load state and terrain scene support before rollout.",
)

TASKS = {LOADED_GRASP_OBSTACLE.task_name: LOADED_GRASP_OBSTACLE}
