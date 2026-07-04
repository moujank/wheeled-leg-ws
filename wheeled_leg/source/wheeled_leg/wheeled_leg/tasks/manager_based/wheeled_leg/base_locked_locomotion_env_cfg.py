# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""训练阶段：锁定机械臂，训练底盘双轮足稳定运动。

本阶段使用机械臂按初始姿态固连在 body 上的 USD 资产，只训练双轮足底盘的平衡、
速度跟踪、轮腿协调和基础地形适应能力。该阶段是后续释放机械臂、倒地恢复、越障、
抓取稳定运动等任务的底层运动能力基础。
"""

from __future__ import annotations

import math

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from . import mdp
from .asset.wheeled import WHEELED_CFG
from .terrains.base_locked import BASE_LOCKED_HARD_TERRAINS_CFG, BASE_LOCKED_TERRAINS_CFG


##
# 场景配置
##


@configclass
class BaseLockedLocomotionSceneCfg(InteractiveSceneCfg):
    """固定机械臂双轮足训练场景。"""

    # 地形：第一阶段先使用碎石/轻度起伏地形，后续可逐步接入更复杂地形课程。
    terrain = TerrainImporterCfg(
        prim_path="/World/terrain",
        terrain_type="generator",
        terrain_generator=BASE_LOCKED_TERRAINS_CFG,
        max_init_terrain_level=5,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/"
            "TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
        debug_vis=False,
    )

    # 机器人：使用机械臂固连版本资产，只暴露轮腿底盘训练所需自由度。
    robot: ArticulationCfg = WHEELED_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # 接触传感器：用于检测机身、髋膝等非期望碰撞，同时支持后续足端腾空时间奖励。
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*",
        debug_vis=False,
        history_length=3,
        track_air_time=True,
    )

    # 环境光照。
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )


##
# MDP 配置
##


@configclass
class CommandsCfg:
    """指令空间配置。"""

    # 底盘速度指令：第一阶段只训练前后速度和朝向跟踪，不训练横向平移。
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 15.0),
        rel_standing_envs=0.02,
        rel_heading_envs=1.0,
        heading_command=True,
        heading_control_stiffness=0.5,
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.5, 0.5),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(0.0, 0.0),
            heading=(-math.pi, math.pi),
        ),
    )


@configclass
class ActionsCfg:
    """动作空间配置。"""

    # 腿部位置控制：策略输出髋关节和膝关节目标位置偏移。
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["Hip.*", "Knee.*"],
        scale=0.5,
        use_default_offset=True,
    )

    # 轮毂速度控制：策略输出左右轮速度指令。
    joint_vel = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=["Hub.*"],
        scale=10.0,
        use_default_offset=True,
    )


@configclass
class ObservationsCfg:
    """观测空间配置。"""

    @configclass
    class PolicyCfg(ObsGroup):
        """Actor 使用的观测。"""

        # 机身角速度。
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            noise=Unoise(n_min=-0.2, n_max=0.2),
        )

        # 重力方向在机身坐标系下的投影，用于感知姿态倾斜。
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )

        # 当前速度指令。
        velocity_commands = ObsTerm(
            func=mdp.generated_commands,
            params={"command_name": "base_velocity"},
        )

        # 关节相对位置。
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )

        # 关节相对速度。
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            noise=Unoise(n_min=-0.1, n_max=0.1),
        )

        # 上一时刻动作。
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        """Critic 使用的特权观测。"""

        # 机身线速度作为特权信息，帮助 critic 估计价值。
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)

        # 机器人各刚体接触状态。
        body_contact = ObsTerm(
            func=mdp.body_contact,
            params={"sensor_cfg": SceneEntityCfg("contact_forces")},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class EventCfg:
    """事件随机化配置。"""

    # 启动时随机化刚体摩擦和恢复系数，提高策略对不同地面材质的鲁棒性。
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.6, 1.0),
            "dynamic_friction_range": (0.4, 0.8),
            "restitution_range": (0.0, 0.005),
            "num_buckets": 64,
        },
    )

    # 启动时随机增加机身质量，模拟建模误差和后续载荷变化。
    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "mass_distribution_params": (0.0, 5.0),
            "operation": "add",
        },
    )

    # reset 时保留外力接口，当前默认不施加外力。
    base_external_force_torque = EventTerm(
        func=mdp.apply_external_force_torque,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "force_range": (0.0, 0.0),
            "torque_range": (-0.0, 0.0),
        },
    )

    # reset 时随机化根节点位姿和速度，避免策略只适应单一初始状态。
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        },
    )

    # reset 时随机化关节初始位置，提升对姿态扰动的适应能力。
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (-0.5, 0.5),
            "velocity_range": (0.0, 0.0),
        },
    )

    # 训练过程中周期性推动机器人，提升抗扰能力。
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    )


@configclass
class RewardsCfg:
    """奖励函数配置。"""

    # 线速度跟踪奖励。
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp,
        weight=50.0,
        params={"command_name": "base_velocity", "std": 0.5},
    )

    # 角速度跟踪奖励，当前权重较低，主要由 heading command 间接约束转向。
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp,
        weight=0.5,
        params={"command_name": "base_velocity", "std": 0.5},
    )

    # 提前终止惩罚。
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)

    # 抑制机身竖直方向速度。
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)

    # 抑制 roll/pitch 方向角速度。
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)

    # 能耗惩罚。
    energy = RewTerm(func=mdp.energy, weight=-1e-3)

    # 关节加速度惩罚，减少动作抖动。
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)

    # 动作变化率惩罚。
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)

    # 姿态保持惩罚，鼓励底盘保持接近水平。
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-20.0)

    # 非期望接触惩罚：髋、膝等部位接触地面会被惩罚。
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*hip.*", ".*knee.*"]),
            "threshold": 1.0,
        },
    )

    # 关节位置限位惩罚。
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-1.0)

    # 腿部关节偏离默认姿态惩罚。
    joint_deviation_legs = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Hip.*", "Knee.*"])},
    )


@configclass
class TerminationsCfg:
    """终止条件配置。"""

    # 时间到达 episode 上限时结束，但不作为失败。
    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    # 机身、髋、膝等关键部位接触地面时判定失败。
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces",
                body_names=["base_link", ".*hip.*", ".*knee.*"],
            ),
            "threshold": 1.0,
        },
    )


##
# 环境配置
##


@configclass
class BaseLockedLocomotionEnvCfg(ManagerBasedRLEnvCfg):
    """固定机械臂的双轮足底盘运动训练配置。"""

    # 场景配置。
    scene: BaseLockedLocomotionSceneCfg = BaseLockedLocomotionSceneCfg(num_envs=4096, env_spacing=2.5)

    # ManagerBased 环境核心配置。
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    commands: CommandsCfg = CommandsCfg()

    def __post_init__(self) -> None:
        """设置仿真步长、episode 长度和地形课程参数。"""
        self.decimation = 4
        self.episode_length_s = 20.0

        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15

        self.scene.contact_forces.update_period = self.sim.dt

        # 第一阶段先关闭地形课程，由简单随机地形打稳底盘基础能力。
        self.scene.terrain.terrain_generator.curriculum = False


@configclass
class BaseLockedLocomotionEnvCfg_PLAY(BaseLockedLocomotionEnvCfg):
    """固定机械臂的双轮足底盘运动测试配置。"""

    def __post_init__(self):
        super().__post_init__()

        # Play 模式减少环境数量，便于可视化观察。
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.episode_length_s = 40.0

        # Play 模式使用较小地形网格，并关闭课程。
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator.num_rows = 5
        self.scene.terrain.terrain_generator.num_cols = 5
        self.scene.terrain.terrain_generator.curriculum = False

        # Play 模式给定固定前进速度，便于观察策略是否稳定。
        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 0.5)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)

        # 测试时关闭观测噪声和外部扰动。
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None


##
# 提升难度版本
##


@configclass
class BaseLockedLocomotionHardSceneCfg(BaseLockedLocomotionSceneCfg):
    """固定机械臂双轮足高难度训练场景。"""

    # 高难度版本引入更强粗糙地形、波浪地形和低台阶，同时开启地形课程。
    terrain = TerrainImporterCfg(
        prim_path="/World/terrain",
        terrain_type="generator",
        terrain_generator=BASE_LOCKED_HARD_TERRAINS_CFG,
        max_init_terrain_level=0,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/"
            "TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
        debug_vis=False,
    )


@configclass
class HardCommandsCfg(CommandsCfg):
    """高难度速度指令配置。"""

    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(6.0, 10.0),
        rel_standing_envs=0.01,
        rel_heading_envs=0.7,
        heading_command=True,
        heading_control_stiffness=0.65,
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.8, 1.2),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(-0.4, 0.4),
            heading=(-math.pi, math.pi),
        ),
    )


@configclass
class HardObservationsCfg(ObservationsCfg):
    """高难度观测配置。"""

    @configclass
    class PolicyCfg(ObservationsCfg.PolicyCfg):
        """高难度 Actor 观测，略微增加传感噪声。"""

        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            noise=Unoise(n_min=-0.3, n_max=0.3),
        )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.08, n_max=0.08),
        )
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            noise=Unoise(n_min=-0.015, n_max=0.015),
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            noise=Unoise(n_min=-0.15, n_max=0.15),
        )

    policy: PolicyCfg = PolicyCfg()


@configclass
class HardEventCfg(EventCfg):
    """高难度事件随机化配置。"""

    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.45, 1.2),
            "dynamic_friction_range": (0.35, 1.0),
            "restitution_range": (0.0, 0.02),
            "num_buckets": 96,
        },
    )
    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "mass_distribution_params": (-1.0, 8.0),
            "operation": "add",
        },
    )
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.8, 0.8), "y": (-0.8, 0.8), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.8, 0.8),
                "y": (-0.4, 0.4),
                "z": (-0.4, 0.4),
                "roll": (-0.8, 0.8),
                "pitch": (-0.8, 0.8),
                "yaw": (-0.8, 0.8),
            },
        },
    )
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (-0.8, 0.8),
            "velocity_range": (-0.3, 0.3),
        },
    )
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(6.0, 10.0),
        params={"velocity_range": {"x": (-1.0, 1.0), "y": (-0.6, 0.6), "yaw": (-0.5, 0.5)}},
    )


@configclass
class HardRewardsCfg(RewardsCfg):
    """高难度奖励配置。"""

    # 在更复杂地形上适度降低速度跟踪权重，给姿态恢复和越障留出优化空间。
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp,
        weight=40.0,
        params={"command_name": "base_velocity", "std": 0.55},
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": 0.55},
    )
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-250.0)
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-1.5)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.08)
    energy = RewTerm(func=mdp.energy, weight=-2.0e-3)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-5.0e-7)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.015)
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-12.0)
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-3.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*hip.*", ".*knee.*"]),
            "threshold": 1.0,
        },
    )
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-1.5)
    joint_deviation_legs = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Hip.*", "Knee.*"])},
    )


@configclass
class BaseLockedLocomotionHardEnvCfg(BaseLockedLocomotionEnvCfg):
    """固定机械臂的双轮足底盘高难度训练配置。"""

    scene: BaseLockedLocomotionHardSceneCfg = BaseLockedLocomotionHardSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: HardObservationsCfg = HardObservationsCfg()
    events: HardEventCfg = HardEventCfg()
    rewards: HardRewardsCfg = HardRewardsCfg()
    commands: HardCommandsCfg = HardCommandsCfg()

    def __post_init__(self) -> None:
        super().__post_init__()

        # 高难度版本启用地形课程，并从低等级逐步增加难度。
        self.scene.terrain.max_init_terrain_level = 0
        self.scene.terrain.terrain_generator.curriculum = True


@configclass
class BaseLockedLocomotionHardEnvCfg_PLAY(BaseLockedLocomotionHardEnvCfg):
    """固定机械臂的双轮足底盘高难度测试配置。"""

    def __post_init__(self):
        super().__post_init__()

        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.episode_length_s = 40.0
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator.num_rows = 5
        self.scene.terrain.terrain_generator.num_cols = 5
        self.scene.terrain.terrain_generator.curriculum = False

        self.commands.base_velocity.ranges.lin_vel_x = (0.8, 0.8)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)

        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
