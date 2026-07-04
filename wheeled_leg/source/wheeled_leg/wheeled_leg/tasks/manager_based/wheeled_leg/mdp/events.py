# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Custom event terms for wheeled-leg training tasks.

Use this module for startup, reset, and interval events such as terrain-level scheduling, robot state
randomization, fall initialization, object spawning, grasp-state randomization, and actuator gain
randomization. Isaac Lab built-in event functions remain available through ``mdp.__init__``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# Add custom event terms here.
# Example future categories:
# - curriculum terrain level updates
# - fall-recovery reset state samplers
# - object mass and pose randomization
# - actuator stiffness/damping randomization

