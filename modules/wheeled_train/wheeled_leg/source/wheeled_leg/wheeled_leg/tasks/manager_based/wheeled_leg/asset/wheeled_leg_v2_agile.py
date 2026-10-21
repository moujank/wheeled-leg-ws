# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Dedicated v2 controllers for obstacle traversal and impulsive leg extension."""

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.utils import configclass

USD_ROOT = Path(__file__).resolve().parents[1] / "usd"


@configclass
class WheeledLegAgileAssetCfg(ArticulationCfg):
    """Independent USD/servo configuration for traversal and jumping.

    The 150 Nm effort cap is retained. Speed limits and servo gains are task
    parameters for simulation; they are not claims about the physical motors.
    """

    spawn = sim_utils.UsdFileCfg(
        usd_path=str(USD_ROOT / "wheeled_leg_v2" / "wheeled_leg_v2.usd"),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False, retain_accelerations=False,
            linear_damping=0.0, angular_damping=0.0,
            max_linear_velocity=1000.0, max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8, solver_velocity_iteration_count=4,
        ),
    )
    init_state = ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.58),
        joint_pos={"HipL": 0.9, "HipR": -0.9, "KneeL": 1.8, "KneeR": -1.8, "Hub.*": 0.0},
    )
    soft_joint_pos_limit_factor = 0.9
    actuators = {
        "hip_act": ImplicitActuatorCfg(
            joint_names_expr=["Hip.*"], effort_limit_sim=150.0,
            velocity_limit_sim=10.0, stiffness=100.0, damping=4.0,
        ),
        "knee_act": ImplicitActuatorCfg(
            joint_names_expr=["Knee.*"], effort_limit_sim=150.0,
            velocity_limit_sim=10.0, stiffness=140.0, damping=5.0,
        ),
        "hub_act": ImplicitActuatorCfg(
            joint_names_expr=["Hub.*"], effort_limit_sim=150.0,
            velocity_limit_sim=12.0, stiffness=0.0, damping=2.0,
        ),
    }


_AGILE_BASE_CFG = WheeledLegAgileAssetCfg()
WHEELED_LEG_V2_OBSTACLE_CFG = _AGILE_BASE_CFG.replace(
    actuators={
        # Track a deliberate crouch/extension without overdamping stair lifts.
        "hip_act": _AGILE_BASE_CFG.actuators["hip_act"].replace(stiffness=120.0, damping=6.0),
        "knee_act": _AGILE_BASE_CFG.actuators["knee_act"].replace(stiffness=180.0, damping=8.0),
        "hub_act": _AGILE_BASE_CFG.actuators["hub_act"].replace(damping=3.0),
    },
)

# Stronger position servos can use the available torque during a short launch.
# Torque remains 150 Nm. Agile simulation speed limits are 10 rad/s for legs
# and 12 rad/s for wheels; these are explicit task settings, not motor ratings.
WHEELED_LEG_V2_JUMP_CFG = _AGILE_BASE_CFG.replace(
    actuators={
        "hip_act": _AGILE_BASE_CFG.actuators["hip_act"].replace(stiffness=140.0, damping=4.0),
        "knee_act": _AGILE_BASE_CFG.actuators["knee_act"].replace(stiffness=220.0, damping=5.0),
        "hub_act": _AGILE_BASE_CFG.actuators["hub_act"].replace(),
    },
)
