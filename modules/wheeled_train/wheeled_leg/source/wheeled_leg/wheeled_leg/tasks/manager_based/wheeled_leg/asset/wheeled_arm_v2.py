"""Training asset for the free-base, twelve-DOF wheeled_arm_v2 robot."""

import math
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg

USD_ROOT = Path(__file__).resolve().parents[1] / "usd"

# Keep the arm above the torso with the elbow folded back toward the shoulder.
# These are joint-space references, not locked joints: PPO controls all six DOFs.
ARM_FOLDED_JOINT_POS = {
    "Arm_R": 0.0,
    "Arm_Y1": -0.65,
    "Arm_Y2": 1.20,
    "Arm_P1": 0.0,
    "Arm_Y3": 0.0,
    "Arm_P2": 0.0,
}

WHEELED_ARM_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(USD_ROOT / "wheeled_arm_v2" / "wheeled_arm_v2.usd"),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            # Match the v1 flat-ground baseline's collision model.
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # Same leg geometry/stance as v1: wheels settle from ~3 cm above ground.
        pos=(0.0, 0.0, 0.58),
        joint_pos={
            "HipL": 0.65,
            "HipR": -0.65,
            "KneeL": 1.3,
            "KneeR": -1.3,
            "Hub.*": 0.0,
            **ARM_FOLDED_JOINT_POS,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "hip_act": ImplicitActuatorCfg(
            joint_names_expr=["Hip.*"],
            # Preserve USD effort limits instead of overriding every motor to 150 Nm.
            effort_limit_sim=120.0,
            velocity_limit_sim=2.0 * math.pi,
            stiffness=20.0,
            damping=1.5,
        ),
        "knee_act": ImplicitActuatorCfg(
            joint_names_expr=["Knee.*"],
            effort_limit_sim=120.0,
            velocity_limit_sim=2.0 * math.pi,
            stiffness=30.0,
            damping=1.5,
        ),
        "hub_act": ImplicitActuatorCfg(
            joint_names_expr=["Hub.*"],
            effort_limit_sim=17.0,
            velocity_limit_sim=2.0 * math.pi,
            stiffness=0.0,
            damping=1.5,
        ),
        "arm_act": ImplicitActuatorCfg(
            joint_names_expr=["Arm.*"],
            effort_limit_sim=27.0,
            velocity_limit_sim=2.0,
            stiffness=40.0,
            damping=6.0,
        ),
    },
)
