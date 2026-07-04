"""Registered MuJoCo task configs mirrored from wheeled_train."""

from __future__ import annotations

from .arm_free_locomotion_env_cfg import TASKS as ARM_FREE_LOCOMOTION_TASKS
from .arm_free_obstacle_env_cfg import TASKS as ARM_FREE_OBSTACLE_TASKS
from .arm_free_recovery_env_cfg import TASKS as ARM_FREE_RECOVERY_TASKS
from .base_locked_locomotion_env_cfg import TASKS as BASE_LOCKED_TASKS
from .cartesian_grasp_env_cfg import TASKS as CARTESIAN_GRASP_TASKS
from .common import BASE_LOCKED_RUNNER, RESERVED_RUNNER, MujocoAssetEnvCfg, merge_task_maps
from .grasp_pose_locomotion_env_cfg import TASKS as GRASP_POSE_TASKS
from .loaded_grasp_obstacle_env_cfg import TASKS as LOADED_GRASP_OBSTACLE_TASKS


TASK_REGISTRY = merge_task_maps(
    BASE_LOCKED_TASKS,
    ARM_FREE_LOCOMOTION_TASKS,
    ARM_FREE_OBSTACLE_TASKS,
    ARM_FREE_RECOVERY_TASKS,
    CARTESIAN_GRASP_TASKS,
    GRASP_POSE_TASKS,
    LOADED_GRASP_OBSTACLE_TASKS,
)


def get_task_config(task_name: str) -> MujocoAssetEnvCfg:
    try:
        return TASK_REGISTRY[task_name]
    except KeyError as exc:
        known = ", ".join(sorted(TASK_REGISTRY))
        raise KeyError(f"Unknown MuJoCo task '{task_name}'. Known tasks: {known}") from exc


__all__ = [
    "BASE_LOCKED_RUNNER",
    "RESERVED_RUNNER",
    "MujocoAssetEnvCfg",
    "TASK_REGISTRY",
    "get_task_config",
]
