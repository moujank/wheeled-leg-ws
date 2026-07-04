# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""训练阶段：释放机械臂，抓取/携载姿态下训练稳定运动、跳跃和越障。

本阶段面向最终集成任务：机器人在机械臂处于抓取或携载姿态时，仍能完成稳定运动、
楼梯/斜坡/粗糙地形通过和跳跃式越障。当前文件先建立机器人侧训练配置；真实物体资产、
抓取接触检测和物体掉落终止条件将在物体 USD/刚体配置确定后接入。
"""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

from . import mdp
from .arm_free_locomotion_env_cfg import ActionsCfg, CommandsCfg, EventCfg, ObservationsCfg, TerminationsCfg
from .asset.wheeled_arm import WHEELED_ARM_CFG
from .terrains.loaded_grasp_obstacle import LOADED_GRASP_OBSTACLE_TERRAINS_CFG


@configclass
class LoadedGraspObstacleSceneCfg(InteractiveSceneCfg):
    """携载/抓取姿态下的越障训练场景。"""

    terrain = TerrainImporterCfg(
        prim_path="/World/terrain",
        terrain_type="generator",
        terrain_generator=LOADED_GRASP_OBSTACLE_TERRAINS_CFG,
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
class RewardsCfg:
    """携载/抓取姿态越障奖励函数。"""

    # 保持基本速度跟踪能力，但比纯运动任务略降低权重，为抓取稳定和越障留出空间。
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp,
        weight=32.0,
        params={"command_name": "base_velocity", "std": 0.6},
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": 0.6},
    )

    # 携载状态下更强调底盘姿态稳定。
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-5.0)
    base_tilt_l2 = RewTerm(func=mdp.base_tilt_l2, weight=-6.0)
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-0.8)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.12)

    # 抑制机械臂和爪部在携载过程中过度摆动。
    arm_joint_vel_l2 = RewTerm(
        func=mdp.joint_vel_l2_selected,
        weight=-0.04,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Arm.*", "Claw"])},
    )
    joint_deviation_arms = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-2.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Arm.*", "Claw"])},
    )

    # 维持动作平滑和能耗约束。
    energy = RewTerm(func=mdp.energy, weight=-0.025)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-5.0e-7)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.018)
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-1.0)

    # 机身、机械臂和爪部撞地都会破坏携载稳定性。
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-8.0,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces",
                body_names=["base_link", ".*hip.*", ".*knee.*", ".*arm.*", ".*claw.*"],
            ),
            "threshold": 1.0,
        },
    )
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-40.0)


@configclass
class LoadedGraspObstacleEnvCfg(ManagerBasedRLEnvCfg):
    """携载/抓取姿态下的稳定运动和越障训练配置。"""

    scene: LoadedGraspObstacleSceneCfg = LoadedGraspObstacleSceneCfg(num_envs=4096, env_spacing=2.5)
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

        # 携载状态下速度略保守，先保证稳定和可通过性。
        self.commands.base_velocity.ranges.lin_vel_x = (0.2, 1.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.15, 0.15)


@configclass
class LoadedGraspObstacleEnvCfg_PLAY(LoadedGraspObstacleEnvCfg):
    """携载/抓取姿态越障测试配置。"""

    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 20
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
