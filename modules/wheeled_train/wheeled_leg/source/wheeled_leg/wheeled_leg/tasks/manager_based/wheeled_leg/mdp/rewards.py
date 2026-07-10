# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import combine_frame_transforms, wrap_to_pi
from isaaclab.sensors import ContactSensor

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# Add custom reward terms in this module.
# Example future categories:
# - staged locomotion rewards
# - recovery progress rewards
# - obstacle traversal rewards
# - grasp stability and object transport rewards


def joint_pos_target_l2(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize joint position deviation from a target value."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # wrap the joint positions to (-pi, pi)
    joint_pos = wrap_to_pi(asset.data.joint_pos[:, asset_cfg.joint_ids])
    # compute the reward
    return torch.sum(torch.square(joint_pos - target), dim=1)

def feet_air_time(
    env: ManagerBasedRLEnv, command_name: str, sensor_cfg: SceneEntityCfg, threshold: float
) -> torch.Tensor:
    """Reward long steps taken by the feet using L2-kernel.

    This function rewards the agent for taking steps that are longer than a threshold. This helps ensure
    that the robot lifts its feet off the ground and takes steps. The reward is computed as the sum of
    the time for which the feet are in the air.

    If the commands are small (i.e. the agent is not supposed to take a step), then the reward is zero.
    """
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # compute the reward
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    reward = torch.sum((last_air_time - threshold) * first_contact, dim=1)
    # no reward for zero command
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    return reward


def feet_air_time_positive_biped(
    env: ManagerBasedRLEnv, command_name: str, threshold: float, sensor_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Reward long steps taken by the feet for bipeds.

    This function rewards the agent for taking steps up to a specified threshold and also keep one foot at
    a time in the air.

    If the commands are small (i.e. the agent is not supposed to take a step), then the reward is zero.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # compute the reward
    air_time = contact_sensor.data.current_air_time[:, sensor_cfg.body_ids]
    contact_time = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids]
    in_contact = contact_time > 0.0
    in_mode_time = torch.where(in_contact, contact_time, air_time)
    single_stance = torch.sum(in_contact.int(), dim=1) == 1
    reward = torch.min(torch.where(single_stance.unsqueeze(-1), in_mode_time, 0.0), dim=1)[0]
    reward = torch.clamp(reward, max=threshold)
    # no reward for zero command
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    return reward


def energy(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset = env.scene[asset_cfg.name]
    reward = torch.norm(torch.abs(asset.data.applied_torque * asset.data.joint_vel), dim=-1)
    return reward


def base_tilt_l2(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize base roll/pitch by using the lateral components of projected gravity."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.projected_gravity_b[:, :2]), dim=1)


def base_height_l2(
    env: ManagerBasedRLEnv,
    target_height: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize deviation of the base height from a nominal target."""
    asset: Articulation = env.scene[asset_cfg.name]
    base_height = asset.data.root_pos_w[:, 2]
    return torch.square(base_height - target_height)


def mirrored_joint_deviation_l1(
    env: ManagerBasedRLEnv,
    left_asset_cfg: SceneEntityCfg,
    right_asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Penalize deviation from mirrored left/right leg configurations."""
    asset: Articulation = env.scene[left_asset_cfg.name]
    left_joint_pos = wrap_to_pi(asset.data.joint_pos[:, left_asset_cfg.joint_ids])
    right_joint_pos = wrap_to_pi(asset.data.joint_pos[:, right_asset_cfg.joint_ids])
    return torch.sum(torch.abs(left_joint_pos + right_joint_pos), dim=1)


def joint_vel_l2_selected(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize the squared velocity of a selected joint subset."""
    asset: Articulation = env.scene[asset_cfg.name]
    joint_vel = asset.data.joint_vel[:, asset_cfg.joint_ids]
    return torch.sum(torch.square(joint_vel), dim=1)


def track_ee_pose_exp(
    env: ManagerBasedRLEnv, 
    std: float, 
    command_name: str, 
    asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Reward end-effector position tracking for base-frame pose commands."""
    asset: Articulation = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    target_pos_b = command[:, :3]
    target_pos_w, _ = combine_frame_transforms(
        asset.data.root_state_w[:, :3], asset.data.root_state_w[:, 3:7], target_pos_b
    )
    ee_pos_w = asset.data.body_state_w[:, asset_cfg.body_ids[0], :3]
    distance_squared = torch.sum(torch.square(target_pos_w - ee_pos_w), dim=1)
    return torch.exp(-distance_squared / std**2)


def track_ee_position_tanh(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Reward end-effector position tracking using a smoother tanh kernel."""
    asset: Articulation = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    target_pos_b = command[:, :3]
    target_pos_w, _ = combine_frame_transforms(
        asset.data.root_state_w[:, :3], asset.data.root_state_w[:, 3:7], target_pos_b
    )
    ee_pos_w = asset.data.body_state_w[:, asset_cfg.body_ids[0], :3]
    distance = torch.norm(target_pos_w - ee_pos_w, dim=1)
    return 1 - torch.tanh(distance / std)
