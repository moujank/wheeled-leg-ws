# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""训练阶段：释放机械臂，训练整机稳定运动。

本阶段使用机械臂自由度完全释放的双轮足模型。训练目标是在机械臂可运动的条件下，
保持底盘速度跟踪、姿态稳定和轮腿协调，同时通过奖励约束机械臂不要破坏整机平衡。
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
from .asset.wheeled_arm import WHEELED_ARM_CFG
from .terrains.arm_free_locomotion import ARM_FREE_LOCOMOTION_TERRAINS_CFG


@configclass
class ArmFreeLocomotionSceneCfg(InteractiveSceneCfg):
    """释放机械臂后的整机运动训练场景。"""

    terrain = TerrainImporterCfg(
        prim_path="/World/terrain",
        terrain_type="generator",
        terrain_generator=ARM_FREE_LOCOMOTION_TERRAINS_CFG,
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

    # 机器人：使用机械臂自由度释放版本资产。
    robot: ArticulationCfg = WHEELED_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

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


@configclass
class CommandsCfg:
    """指令空间配置。"""

    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(8.0, 12.0),
        rel_standing_envs=0.02,
        rel_heading_envs=0.0,
        heading_command=False,
        heading_control_stiffness=0.5,
        debug_vis=False,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.2, 1.0),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(-0.2, 0.2),
            heading=(0.0, 0.0),
        ),
    )


@configclass
class ActionsCfg:
    """动作空间配置。"""

    joint_pos_leg = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["Hip.*", "Knee.*"],
        scale=0.5,
        use_default_offset=True,
    )
    joint_vel = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=["Hub.*"],
        scale=10.0,
        use_default_offset=True,
    )
    joint_pos_arm = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["Arm.*", "Claw"],
        scale=0.15,
        use_default_offset=True,
    )


@configclass
class ObservationsCfg:
    """观测空间配置。"""

    @configclass
    class PolicyCfg(ObsGroup):
        """Actor 使用的观测。"""

        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.1, n_max=0.1))
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
            self.history_length = 3
            self.flatten_history_dim = True

    @configclass
    class CriticCfg(ObsGroup):
        """Critic 使用的特权观测。"""

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        body_contact = ObsTerm(func=mdp.body_contact, params={"sensor_cfg": SceneEntityCfg("contact_forces")})

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class EventCfg:
    """事件随机化配置。"""

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
            "pose_range": {"x": (-0.2, 0.2), "y": (-0.2, 0.2), "yaw": (-0.5, 0.5)},
            "velocity_range": {
                "x": (-0.1, 0.1),
                "y": (-0.1, 0.1),
                "z": (-0.1, 0.1),
                "roll": (-0.1, 0.1),
                "pitch": (-0.1, 0.1),
                "yaw": (-0.1, 0.1),
            },
        },
    )
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={"position_range": (0.9, 1.1), "velocity_range": (0.0, 0.0)},
    )
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(15.0, 20.0),
        params={"velocity_range": {"x": (-0.2, 0.2), "y": (-0.2, 0.2)}},
    )


@configclass
class RewardsCfg:
    """奖励函数配置。"""

    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp, weight=40.0, params={"command_name": "base_velocity", "std": 0.6}
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp, weight=2.0, params={"command_name": "base_velocity", "std": 0.6}
    )
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-20.0)
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-1.0)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.15)
    energy = RewTerm(func=mdp.energy, weight=-0.03)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-5.0e-7)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.015)
    arm_joint_vel_l2 = RewTerm(
        func=mdp.joint_vel_l2_selected,
        weight=-0.02,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Arm.*", "Claw"])},
    )
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-3.0)
    base_tilt_l2 = RewTerm(func=mdp.base_tilt_l2, weight=-4.0)
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-6.0,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", body_names=["base_link", ".*hip.*", ".*knee.*", ".*arm.*", ".*claw.*"]
            ),
            "threshold": 1.0,
        },
    )
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-1.0)
    joint_deviation_legs = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Hip.*", "Knee.*"])},
    )
    joint_deviation_arms = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-1.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Arm.*", "Claw"])},
    )


@configclass
class TerminationsCfg:
    """终止条件配置。"""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", body_names=["base_link", ".*hip.*", ".*knee.*", ".*arm.*", ".*claw.*"]
            ),
            "threshold": 1.0,
        },
    )


@configclass
class ArmFreeLocomotionEnvCfg(ManagerBasedRLEnvCfg):
    """释放机械臂后的整机稳定运动训练配置。"""

    scene: ArmFreeLocomotionSceneCfg = ArmFreeLocomotionSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    commands: CommandsCfg = CommandsCfg()

    def __post_init__(self) -> None:
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15
        self.scene.contact_forces.update_period = self.sim.dt
        self.scene.terrain.terrain_generator.curriculum = True


@configclass
class ArmFreeLocomotionEnvCfg_PLAY(ArmFreeLocomotionEnvCfg):
    """释放机械臂后的整机稳定运动测试配置。"""

    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.episode_length_s = 40.0
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator.num_rows = 5
        self.scene.terrain.terrain_generator.num_cols = 5
        self.scene.terrain.terrain_generator.curriculum = False
        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 0.5)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
