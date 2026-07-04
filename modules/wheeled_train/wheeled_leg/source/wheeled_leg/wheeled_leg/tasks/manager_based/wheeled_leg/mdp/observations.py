# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Custom observation terms for wheeled-leg training tasks."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.utils.math import quat_apply_inverse

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def body_contact(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Return a boolean tensor that marks bodies currently in contact."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    return contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0.001


def object_position_in_robot_frame(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Return the commanded target position expressed in the robot base frame."""
    asset: Articulation = env.scene[asset_cfg.name]

    target_pos_w = env.command_manager.get_command(command_name)[:, :3]
    base_pos_w = asset.data.root_pos_w
    base_quat_w = asset.data.root_quat_w

    return quat_apply_inverse(base_quat_w, target_pos_w - base_pos_w)


def ee_position_error_in_robot_frame(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Return the end-effector position error in the robot base frame."""
    asset: Articulation = env.scene[asset_cfg.name]

    target_pos_w = env.command_manager.get_command(command_name)[:, :3]
    ee_pos_w = asset.data.body_pos_w[:, asset_cfg.body_ids[0], :]

    base_pos_w = asset.data.root_pos_w
    base_quat_w = asset.data.root_quat_w

    target_pos_b = quat_apply_inverse(base_quat_w, target_pos_w - base_pos_w)
    ee_pos_b = quat_apply_inverse(base_quat_w, ee_pos_w - base_pos_w)
    return target_pos_b - ee_pos_b


# Add new custom observation terms here.
# Example future categories:
# - object pose in robot frame
# - grasp state and contact state
# - terrain preview features
# - recovery orientation features

