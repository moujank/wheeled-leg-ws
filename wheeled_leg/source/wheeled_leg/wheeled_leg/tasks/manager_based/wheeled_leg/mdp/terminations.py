# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Custom termination terms for wheeled-leg training tasks.

Use this module for task-specific done conditions, including unrecoverable falls, grasp loss, object drop,
end-effector workspace violations, obstacle traversal failure, or curriculum-specific safety bounds.
Isaac Lab built-in termination functions remain available through ``mdp.__init__``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# Add custom termination terms here.
# Example future categories:
# - grasp object dropped
# - end-effector target permanently unreachable
# - recovery timeout after fallen initialization
# - obstacle traversal failure

