# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Custom action terms for wheeled-leg training tasks.

Use this module for action preprocessing or custom action terms that are not covered by Isaac Lab's
built-in joint position, joint velocity, or joint effort action terms. This is the right place for future
dynamic stiffness/damping action logic if actuator gains become policy-controlled or schedule-controlled.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# Add custom action terms here.
# Example future categories:
# - policy-controlled actuator stiffness and damping
# - action scaling or masking for staged curricula
# - arm/base action blending during recovery or grasp tasks

