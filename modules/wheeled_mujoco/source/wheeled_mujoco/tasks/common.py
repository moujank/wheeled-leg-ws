"""Shared MuJoCo task configuration types."""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Mapping

from wheeled_mujoco.paths import MJCF_ROOT, TRAIN_LOG_ROOT, policy_path


@dataclass(frozen=True)
class CommandRangeCfg:
    """Body-frame velocity command ranges mirrored from training PLAY configs."""

    lin_vel_x: tuple[float, float]
    lin_vel_y: tuple[float, float] = (0.0, 0.0)
    ang_vel_z: tuple[float, float] = (0.0, 0.0)
    heading: tuple[float, float] | None = (0.0, 0.0)


@dataclass(frozen=True)
class MujocoAssetEnvCfg:
    """MuJoCo asset/environment config for a trained task."""

    task_name: str
    train_env_cfg: str
    log_name: str
    mjcf_path: Path
    policy_path: Path
    command: CommandRangeCfg
    robot_variant: str
    runner_kind: str
    notes: str = ""

    def with_backend(self, backend: str) -> "MujocoAssetEnvCfg":
        return replace(self, policy_path=policy_path(self.log_name, backend=backend))


BASE_LOCKED_RUNNER = "base_locked_27d_policy"
RESERVED_RUNNER = "reserved_task_runner"


def leg_task(
    task_name: str,
    train_env_cfg: str,
    log_name: str,
    command: CommandRangeCfg,
    runner_kind: str = RESERVED_RUNNER,
    notes: str = "",
) -> MujocoAssetEnvCfg:
    return MujocoAssetEnvCfg(
        task_name=task_name,
        train_env_cfg=train_env_cfg,
        log_name=log_name,
        mjcf_path=MJCF_ROOT / "wheeled_leg.xml",
        policy_path=TRAIN_LOG_ROOT / log_name / "best" / "exported" / "policy.pt",
        command=command,
        robot_variant="wheeled_leg",
        runner_kind=runner_kind,
        notes=notes,
    )


def arm_task(
    task_name: str,
    train_env_cfg: str,
    log_name: str,
    command: CommandRangeCfg,
    runner_kind: str = RESERVED_RUNNER,
    notes: str = "",
) -> MujocoAssetEnvCfg:
    return MujocoAssetEnvCfg(
        task_name=task_name,
        train_env_cfg=train_env_cfg,
        log_name=log_name,
        mjcf_path=MJCF_ROOT / "wheeled_arm.xml",
        policy_path=TRAIN_LOG_ROOT / log_name / "best" / "exported" / "policy.pt",
        command=command,
        robot_variant="wheeled_arm",
        runner_kind=runner_kind,
        notes=notes,
    )


def merge_task_maps(*maps: Mapping[str, MujocoAssetEnvCfg]) -> dict[str, MujocoAssetEnvCfg]:
    merged: dict[str, MujocoAssetEnvCfg] = {}
    for task_map in maps:
        overlap = set(merged).intersection(task_map)
        if overlap:
            raise ValueError(f"Duplicate MuJoCo task names: {sorted(overlap)}")
        merged.update(task_map)
    return merged
