# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Custom command terms for wheeled-leg training tasks.

Use this module for command generators and command-related helper functions, such as base velocity
commands, end-effector pose commands, recovery pose commands, obstacle targets, and object grasp targets.
Isaac Lab built-in command terms are re-exported from ``isaaclab.envs.mdp`` by this package's
``mdp.__init__``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs.mdp.commands import UniformVelocityCommand, UniformVelocityCommandCfg
from isaaclab.utils import configclass

from .traversal import traversal_flat_weight

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class TraversalVelocityCommand(UniformVelocityCommand):
    """Unchanged velocity commands with episode-duration-normalized gait metrics."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._hub_ids, _ = env.scene.sensors['contact_forces'].find_bodies('Hub.*')
        self._leg_ids, _ = self.robot.find_joints(['Hip.*', 'Knee.*'])
        self._motion_samples = torch.zeros(self.num_envs, device=self.device)
        for name in ('episode_vel_xy_error', 'forward_speed', 'vertical_speed_abs',
                     'leg_speed_abs', 'dual_wheel_air_fraction', 'flat_air_fraction', 'flat_fraction'):
            self.metrics[name] = torch.zeros(self.num_envs, device=self.device)

    def _update_metrics(self):
        super()._update_metrics()
        active = self._env.episode_length_buf > 0
        self._motion_samples += active
        air = (self._env.scene.sensors['contact_forces'].data.current_air_time[:, self._hub_ids] > 0.016).all(dim=1)
        flat = traversal_flat_weight(self._env) > 0.9
        data = self.robot.data
        values = {
            'episode_vel_xy_error': torch.linalg.vector_norm(self.command[:, :2] - data.root_lin_vel_b[:, :2], dim=1),
            'forward_speed': data.root_lin_vel_w[:, 0],
            'vertical_speed_abs': data.root_lin_vel_w[:, 2].abs(),
            'leg_speed_abs': data.joint_vel[:, self._leg_ids].abs().mean(dim=1),
            'dual_wheel_air_fraction': air.float(),
            'flat_air_fraction': (flat & air).float(),
            'flat_fraction': flat.float(),
        }
        for name, value in values.items():
            self.metrics[name] += active * (value - self.metrics[name]) / self._motion_samples.clamp(min=1)

    def reset(self, env_ids=None):
        extras = super().reset(env_ids)
        self._motion_samples[slice(None) if env_ids is None else env_ids] = 0
        return extras


@configclass
class TraversalVelocityCommandCfg(UniformVelocityCommandCfg):
    class_type: type = TraversalVelocityCommand

