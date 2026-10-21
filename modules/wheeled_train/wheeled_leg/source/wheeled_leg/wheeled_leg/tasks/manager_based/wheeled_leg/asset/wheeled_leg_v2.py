import isaaclab.sim as sim_utils
from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.actuators import ImplicitActuatorCfg
import math
from pathlib import Path

USD_ROOT = Path(__file__).resolve().parents[1] / "usd"

WHEELED_CFG = ArticulationCfg(
    #prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(USD_ROOT / "wheeled_leg_v2" / "wheeled_leg_v2.usd"),
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
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # 当前 USD 的髋、膝关节限位为零；先使用合法零位完成运行验证。
        # 修复资产限位后，再恢复和验证屈腿初始姿态。
        pos=(0.0, 0.0, 0.7),
        joint_pos={
            "HipL": 0.65,
            "HipR": -0.65,
            "KneeL": 1.3,
            "KneeR": -1.3,
            "Hub.*": 0.0,
        }
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "hip_act": ImplicitActuatorCfg(
            joint_names_expr=["Hip.*"],
            effort_limit=150.0,
            velocity_limit=math.radians(360.0),
            stiffness=20.0,
            damping=1.5,
        ),
        "knee_act": ImplicitActuatorCfg(
            joint_names_expr=["Knee.*"],
            effort_limit=150.0,
            velocity_limit=math.radians(360.0),
            stiffness=30.0,
            damping=1.5,
        ),
        "hub_act": ImplicitActuatorCfg(
            joint_names_expr=["Hub.*"],
            effort_limit=150.0,
            velocity_limit=math.radians(360.0),
            stiffness=0.0,
            damping=1.5,
        ),
    },
)

# Dedicated instance: keep the trained robot's pose and physical motor limits.
WHEELED_LEG_OBSTACLE_JUMP_CFG = WHEELED_CFG.replace(
    init_state=WHEELED_CFG.init_state.replace(
        pos=(0.0, 0.0, 0.58),
        joint_pos={
            "HipL": 0.65,
            "HipR": -0.65,
            "KneeL": 1.30,
            "KneeR": -1.30,
            "Hub.*": 0.0,
        },
    ),
    actuators={
        name: actuator.replace(
            velocity_limit=None,
            velocity_limit_sim=2.0 * math.pi,
            damping=2.5 if name in ("hip_act", "knee_act") else actuator.damping,
        )
        for name, actuator in WHEELED_CFG.actuators.items()
    },
)
