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

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# Add custom command terms here.
# Example future categories:
# - whole-body velocity command schedules
# - fall-recovery initial pose commands
# - Cartesian grasp pose command generators
# - object-relative target commands

