"""Initial task-switching utilities for future coupled MuJoCo tasks."""

from __future__ import annotations

from dataclasses import dataclass

from wheeled_mujoco.tasks import MujocoAssetEnvCfg, TASK_REGISTRY, get_task_config


@dataclass(frozen=True)
class TaskTransition:
    """Description of a future transition between two sim2sim task contexts."""

    source_task: str
    target_task: str
    blend_time_s: float = 0.0
    reset_required: bool = False
    notes: str = ""


class TaskSwitcher:
    """Small registry wrapper for selecting MuJoCo task configs."""

    def __init__(self, initial_task: str = "wheeled_base_locked"):
        self._active_task_name = initial_task
        get_task_config(initial_task)

    @property
    def active_task_name(self) -> str:
        return self._active_task_name

    @property
    def active_config(self) -> MujocoAssetEnvCfg:
        return get_task_config(self._active_task_name)

    def available_tasks(self) -> tuple[str, ...]:
        return tuple(sorted(TASK_REGISTRY))

    def switch_to(self, target_task: str, blend_time_s: float = 0.0, reset_required: bool = False) -> TaskTransition:
        get_task_config(target_task)
        transition = TaskTransition(
            source_task=self._active_task_name,
            target_task=target_task,
            blend_time_s=blend_time_s,
            reset_required=reset_required,
        )
        self._active_task_name = target_task
        return transition
