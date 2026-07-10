# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""训练阶段：释放机械臂，在抓取准备姿态下训练平衡稳定运动。

本阶段不引入笛卡尔目标点，而是要求机器人在机械臂处于抓取准备姿态或接近抓取姿态时，
仍然保持底盘速度跟踪和姿态稳定，为后续末端控制和真实抓取建立整机平衡能力。
"""

from __future__ import annotations

import math

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from . import mdp
from .arm_free_locomotion_env_cfg import ActionsCfg, EventCfg, TerminationsCfg
from .asset.wheeled_arm import WHEELED_ARM_CFG
from .terrains.grasp_pose import GRASP_POSE_TERRAINS_CFG


@configclass
class GraspPoseLocomotionSceneCfg(InteractiveSceneCfg):
    """抓取准备姿态下的稳定运动训练场景。"""

    terrain = TerrainImporterCfg(
        prim_path="/World/terrain",
        terrain_type="generator",
        terrain_generator=GRASP_POSE_TERRAINS_CFG,
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

    robot: ArticulationCfg = WHEELED_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    target_visual = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Target",
        spawn=sim_utils.SphereCfg(
            radius=0.02,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
        ),
    )

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
    """抓取准备姿态下的指令空间配置。"""

    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(8.0, 12.0),
        rel_standing_envs=0.1,
        rel_heading_envs=0.0,
        heading_command=False,
        heading_control_stiffness=0.5,
        debug_vis=False,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 0.8),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(-0.2, 0.2),
            heading=(0.0, 0.0),
        ),
    )
    ee_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name="claw_Link",
        resampling_time_range=(4.0, 6.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            # 缩小目标点采样范围，保证不同抓取准备姿态下都能覆盖。
            pos_x=(0.36, 0.50),
            pos_y=(-0.08, 0.08),
            pos_z=(0.18, 0.30),
            # 目标坐标系默认与 claw_Link 末端抓取朝向一致，并加入小角度扰动。
            roll=(-0.10, 0.10),
            pitch=(math.pi * 0.5 - 0.10, math.pi * 0.5 + 0.10),
            yaw=(-0.12, 0.12),
        ),
    )


@configclass
class ObservationsCfg:
    """抓取准备姿态下的观测空间配置。"""

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
        ee_position_to_target = ObsTerm(
            func=mdp.ee_position_error_in_robot_frame,
            params={"asset_cfg": SceneEntityCfg("robot", body_names=["claw_Link"]), "command_name": "ee_pose"},
        )

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
class RewardsCfg:
    """抓取准备姿态下的稳定运动奖励函数。"""

    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp, weight=38.0, params={"command_name": "base_velocity", "std": 0.6}
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp, weight=1.5, params={"command_name": "base_velocity", "std": 0.6}
    )
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-20.0)
    approach_ee_target_exp = RewTerm(
        func=mdp.track_ee_pose_exp,
        weight=16.0,
        params={"command_name": "ee_pose", "std": 0.28, "asset_cfg": SceneEntityCfg("robot", body_names=["claw_Link"])},
    )
    approach_ee_target_tanh = RewTerm(
        func=mdp.track_ee_position_tanh,
        weight=24.0,
        params={"command_name": "ee_pose", "std": 0.16, "asset_cfg": SceneEntityCfg("robot", body_names=["claw_Link"])},
    )
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-1.0)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.15)
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-4.0)
    base_tilt_l2 = RewTerm(func=mdp.base_tilt_l2, weight=-5.0)
    energy = RewTerm(func=mdp.energy, weight=-0.025)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-5.0e-7)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.015)
    # 机械臂姿态不要剧烈偏离默认抓取准备姿态。
    joint_deviation_arms = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-2.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Arm.*", "Claw"])},
    )
    joint_deviation_legs = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Hip.*", "Knee.*"])},
    )
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


@configclass
class GraspPoseLocomotionEnvCfg(ManagerBasedRLEnvCfg):
    """抓取准备姿态下的稳定运动训练配置。"""

    scene: GraspPoseLocomotionSceneCfg = GraspPoseLocomotionSceneCfg(num_envs=4096, env_spacing=2.5)
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
class GraspPoseLocomotionEnvCfg_PLAY(GraspPoseLocomotionEnvCfg):
    """抓取准备姿态下的稳定运动测试配置。"""

    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 30
        self.scene.env_spacing = 2.5
        self.episode_length_s = 40.0
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator.num_rows = 5
        self.scene.terrain.terrain_generator.num_cols = 5
        self.scene.terrain.terrain_generator.curriculum = False
        self.commands.base_velocity.ranges.lin_vel_x = (0.4, 0.4)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)
        self.commands.ee_pose.ranges.pos_x = (0.43, 0.43)
        self.commands.ee_pose.ranges.pos_y = (0.0, 0.0)
        self.commands.ee_pose.ranges.pos_z = (0.24, 0.24)
        self.commands.ee_pose.ranges.roll = (0.0, 0.0)
        self.commands.ee_pose.ranges.pitch = (math.pi * 0.5, math.pi * 0.5)
        self.commands.ee_pose.ranges.yaw = (0.0, 0.0)
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
