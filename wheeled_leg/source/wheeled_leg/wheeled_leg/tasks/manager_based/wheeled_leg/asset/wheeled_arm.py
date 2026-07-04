import isaaclab.sim as sim_utils
from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.actuators import ImplicitActuatorCfg
import math
from pathlib import Path

USD_ROOT = Path(__file__).resolve().parents[1] / "usd"

WHEELED_ARM_CFG = ArticulationCfg(
    #prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(USD_ROOT / "usd_arm" / "wheeled_arm.usd"),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.05,
            angular_damping=0.05,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=3.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=8,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.34),
        joint_pos={
            "HipL": 0.0,
            "HipR": 0.0, 
            "KneeL": 0.0,
            "KneeR": 0.0,
            "Hub.*": 0.0,
            "Arm.*": 0.0,
            "Claw": 0.0,
        }
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "hip_act": ImplicitActuatorCfg(
            joint_names_expr=["Hip.*"],
            effort_limit=150.0,
            velocity_limit=math.radians(360.0),
            stiffness=40.0,
            damping=1.5,
        ),
        "knee_act": ImplicitActuatorCfg(
            joint_names_expr=["Knee.*"],
            effort_limit=150.0,
            velocity_limit=math.radians(360.0),
            stiffness=60.0,
            damping=1.5,
        ),
        "hub_act": ImplicitActuatorCfg(
            joint_names_expr=["Hub.*"],
            effort_limit=150.0,
            velocity_limit=math.radians(360.0),
            stiffness=0.0,
            damping=0.5,
        ), 
        "arm_act": ImplicitActuatorCfg(
            joint_names_expr=["Arm.*"],
            effort_limit=150.0,
            velocity_limit=math.radians(360.0),
            stiffness=16.0,
            damping=6.0,
        ),
        "claw_act": ImplicitActuatorCfg(
            joint_names_expr=["Claw"],
            effort_limit=120.0,
            velocity_limit=math.radians(360.0),
            stiffness=6.0,
            damping=4.0,
        ),
    },
)
