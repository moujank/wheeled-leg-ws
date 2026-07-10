"""MuJoCo config for the base_locked_locomotion training tasks."""

from __future__ import annotations

from wheeled_mujoco.tasks.common import BASE_LOCKED_RUNNER, CommandRangeCfg, leg_task


BASE_LOCKED_LOCOMOTION = leg_task(
    task_name="wheeled_base_locked",
    train_env_cfg="base_locked_locomotion_env_cfg.py:BaseLockedLocomotionEnvCfg_PLAY",
    log_name="wheeled_base_locked",
    command=CommandRangeCfg(lin_vel_x=(0.5, 0.5), lin_vel_y=(0.0, 0.0), heading=(0.0, 0.0)),
    runner_kind=BASE_LOCKED_RUNNER,
    notes="Aligned to Isaac Play defaults: 0.5 m/s forward command with leg scale 0.35 and wheel scale 6.0.",
)

BASE_LOCKED_LOCOMOTION_HARD = leg_task(
    task_name="wheeled_base_locked_hard",
    train_env_cfg="base_locked_locomotion_env_cfg.py:BaseLockedLocomotionHardEnvCfg_PLAY",
    log_name="wheeled_base_locked_hard",
    command=CommandRangeCfg(lin_vel_x=(2.0, 2.0), lin_vel_y=(0.0, 0.0), heading=(0.0, 0.0)),
    runner_kind=BASE_LOCKED_RUNNER,
    notes="Aligned to Isaac Play hard-task defaults; requires separate validation before use as a stable baseline.",
)

TASKS = {
    BASE_LOCKED_LOCOMOTION.task_name: BASE_LOCKED_LOCOMOTION,
    BASE_LOCKED_LOCOMOTION_HARD.task_name: BASE_LOCKED_LOCOMOTION_HARD,
}
