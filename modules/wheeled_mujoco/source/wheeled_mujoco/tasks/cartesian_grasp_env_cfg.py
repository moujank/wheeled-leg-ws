"""MuJoCo config for cartesian_grasp."""

from __future__ import annotations

from wheeled_mujoco.tasks.common import CommandRangeCfg, arm_task


CARTESIAN_GRASP = arm_task(
    task_name="wheeled_cartesian_grasp",
    train_env_cfg="cartesian_grasp_env_cfg.py:CartesianGraspEnvCfg_PLAY",
    log_name="wheeled_cartesian_grasp",
    command=CommandRangeCfg(lin_vel_x=(0.0, 0.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.0, 0.0), heading=None),
    notes="Grasp command channels are not yet mapped to MuJoCo; this config only reserves asset and policy paths.",
)

TASKS = {CARTESIAN_GRASP.task_name: CARTESIAN_GRASP}
