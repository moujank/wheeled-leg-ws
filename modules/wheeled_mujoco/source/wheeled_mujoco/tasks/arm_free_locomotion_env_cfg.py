"""MuJoCo config for arm_free_locomotion."""

from __future__ import annotations

from wheeled_mujoco.tasks.common import CommandRangeCfg, arm_task


ARM_FREE_LOCOMOTION = arm_task(
    task_name="wheeled_arm_free_locomotion",
    train_env_cfg="arm_free_locomotion_env_cfg.py:ArmFreeLocomotionEnvCfg_PLAY",
    log_name="wheeled_arm_free_locomotion",
    command=CommandRangeCfg(lin_vel_x=(0.5, 0.5), lin_vel_y=(0.0, 0.0), heading=(0.0, 0.0)),
    notes="Arm-free locomotion uses wheeled_arm.xml. Observation/action deployment still needs a dedicated runner.",
)

TASKS = {ARM_FREE_LOCOMOTION.task_name: ARM_FREE_LOCOMOTION}
