# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""训练阶段：锁定机械臂，训练底盘双轮足稳定运动的高难度版本。

该文件为完全独立的 hard 配置，不继承 base_locked_locomotion_env_cfg.py 中的任何类，
便于后续单独维护高难度课程学习版本。
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
from .terrains.base_locked import BASE_LOCKED_HARD_TERRAINS_CFG

##
# 场景配置
##


@configclass
class BaseLockedLocomotionHardSceneCfg(InteractiveSceneCfg):
    """固定机械臂双轮足高难度训练场景。"""

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
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
        debug_vis=False,
    )

    robot: ArticulationCfg = WHEELED_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*",
        debug_vis=False,
        history_length=3,
        track_air_time=True,
    )

    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )


##
# MDP settings
##


@configclass
class HardCommandsCfg:
    """高难度速度指令配置。"""

    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(8.0, 12.0),
        rel_standing_envs=0.0,
        rel_heading_envs=0.0,
        heading_command=False,
        heading_control_stiffness=0.4,
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.2, 2.0),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(0.0, 0.0),
            heading=(0.0, 0.0),
        ),
    )


@configclass
class HardActionsCfg:
    """高难度动作配置。"""

    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["Hip.*", "Knee.*"],
        scale=0.4,
        use_default_offset=True,
    )
    joint_vel = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=["Hub.*"],
        scale=8.0,
        use_default_offset=True,
    )


@configclass
class HardObservationsCfg:
    """高难度观测配置。"""

    @configclass
    class PolicyCfg(ObsGroup):
        """Actor 观测。"""

        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel,
            noise=Unoise(n_min=-0.08, n_max=0.08),
        )
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            noise=Unoise(n_min=-0.15, n_max=0.15),
        )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.03, n_max=0.03),
        )
        velocity_commands = ObsTerm(
            func=mdp.generated_commands,
            params={"command_name": "base_velocity"},
        )
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            noise=Unoise(n_min=-0.08, n_max=0.08),
        )
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        """Critic 观测。"""

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
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
class HardEventCfg:
    """高难度事件随机化配置。"""

    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.7, 1.0),
            "dynamic_friction_range": (0.5, 0.8),
            "restitution_range": (0.0, 0.005),
            "num_buckets": 64,
        },
    )

    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "mass_distribution_params": (0.0, 2.0),
            "operation": "add",
        },
    )

    base_external_force_torque = EventTerm(
        func=mdp.apply_external_force_torque,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "force_range": (0.0, 0.0),
            "torque_range": (-0.0, 0.0),
        },
    )

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.2, 0.2),
                "y": (-0.1, 0.1),
                "z": (-0.1, 0.1),
                "roll": (-0.2, 0.2),
                "pitch": (-0.2, 0.2),
                "yaw": (-0.25, 0.25),
            },
        },
    )

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (-0.5, 0.5),
            "velocity_range": (0.0, 0.0),
        },
    )

    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(16.0, 24.0),
        params={"velocity_range": {"x": (-0.2, 0.2), "y": (-0.1, 0.1), "yaw": (-0.15, 0.15)}},
    )


@configclass
class HardRewardsCfg:
    """高难度奖励配置。

    调整方向基于上一轮训练分析：
    1. 不再继续显著提高 tracking 奖励上限；
    2. 优先压制垂向弹跳、姿态异常和动作突变；
    3. 适度加强终止代价，避免课程后段用大动作硬扛地形。
    """

    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp,
        weight=52.0,
        params={"command_name": "base_velocity", "std": 0.8},
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp,
        weight=4.0,
        params={"command_name": "base_velocity", "std": 0.5},
    )

    termination_penalty = RewTerm(
        func=mdp.is_terminated,
        weight=-250.0,
    )
    lin_vel_z_l2 = RewTerm(
        func=mdp.lin_vel_z_l2,
        weight=-3.5,
    )
    ang_vel_xy_l2 = RewTerm(
        func=mdp.ang_vel_xy_l2,
        weight=-0.12,
    )
    energy = RewTerm(
        func=mdp.energy,
        weight=-1.5e-3,
    )
    dof_acc_l2 = RewTerm(
        func=mdp.joint_acc_l2,
        weight=-4.0e-7,
    )
    action_rate_l2 = RewTerm(
        func=mdp.action_rate_l2,
        weight=-0.03,
    )
    flat_orientation_l2 = RewTerm(
        func=mdp.flat_orientation_l2,
        weight=-28.0,
    )
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-2.0,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*hip.*", ".*knee.*"]), "threshold": 1.0},
    )
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-2.0,
    )
    joint_deviation_legs = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.3,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Hip.*", "Knee.*"])},
    )
    leg_symmetry = RewTerm(
        func=mdp.mirrored_joint_deviation_l1,
        weight=-0.2,
        params={
            "left_asset_cfg": SceneEntityCfg("robot", joint_names=["HipL", "KneeL"]),
            "right_asset_cfg": SceneEntityCfg("robot", joint_names=["HipR", "KneeR"]),
        },
    )


@configclass
class HardTerminationsCfg:
    """高难度终止配置。"""

    time_out = DoneTerm(
        func=mdp.time_out,
        time_out=True,
    )
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["base_link", ".*hip.*", ".*knee.*"]),
            "threshold": 1.0,
        },
    )


##
# 环境配置
##


@configclass
class BaseLockedLocomotionHardEnvCfg(ManagerBasedRLEnvCfg):
    """固定机械臂的双轮足底盘高难度训练配置。"""

    scene: BaseLockedLocomotionHardSceneCfg = BaseLockedLocomotionHardSceneCfg(num_envs=4096, env_spacing=2.5)

    observations: HardObservationsCfg = HardObservationsCfg()
    actions: HardActionsCfg = HardActionsCfg()
    events: HardEventCfg = HardEventCfg()
    rewards: HardRewardsCfg = HardRewardsCfg()
    terminations: HardTerminationsCfg = HardTerminationsCfg()
    commands: HardCommandsCfg = HardCommandsCfg()

    def __post_init__(self) -> None:
        self.decimation = 4
        self.episode_length_s = 24.0

        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15

        self.scene.contact_forces.update_period = self.sim.dt

        # Hard 版本启用课程学习地形，并从最低等级开始逐步提升难度。
        self.scene.terrain.max_init_terrain_level = 0
        self.scene.terrain.terrain_generator.curriculum = True


@configclass
class BaseLockedLocomotionHardEnvCfg_PLAY(BaseLockedLocomotionHardEnvCfg):
    """固定机械臂的双轮足底盘高难度测试配置。"""

    torque_logging_enabled: bool = True
    torque_log_folder_name: str = "play_joint_torque"
    torque_log_env_index: int = 0
    torque_log_flush_interval: int = 50

    def __post_init__(self):
        super().__post_init__()

        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.episode_length_s = 40.0

        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator.num_rows = 5
        self.scene.terrain.terrain_generator.num_cols = 5
        self.scene.terrain.terrain_generator.curriculum = False

        self.commands.base_velocity.ranges.lin_vel_x = (2.0, 2.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)

        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
