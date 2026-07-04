# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""训练阶段：释放机械臂，训练倒地恢复。

本阶段使用机械臂自由度释放的整机模型，从较大姿态扰动、倾倒姿态或低稳定状态中恢复到
可控直立运动状态。该任务不把机身接触地面直接作为失败条件，否则无法学习恢复过程。
"""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

from . import mdp
from .arm_free_locomotion_env_cfg import ActionsCfg, CommandsCfg, ObservationsCfg
from .asset.wheeled_arm import WHEELED_ARM_CFG
from .terrains.recovery import RECOVERY_TERRAINS_CFG


@configclass
class ArmFreeRecoverySceneCfg(InteractiveSceneCfg):
    """倒地恢复训练场景。"""

    terrain = TerrainImporterCfg(
        prim_path="/World/terrain",
        terrain_type="generator",
        terrain_generator=RECOVERY_TERRAINS_CFG,
        max_init_terrain_level=4,
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
class EventCfg:
    """倒地恢复事件随机化。"""

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
    # reset 时允许较大 roll/pitch 扰动，用于学习从倾倒状态恢复。
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": (-0.2, 0.2),
                "y": (-0.2, 0.2),
                "z": (-0.05, 0.05),
                "roll": (-1.2, 1.2),
                "pitch": (-1.2, 1.2),
                "yaw": (-3.14, 3.14),
            },
            "velocity_range": {
                "x": (-0.2, 0.2),
                "y": (-0.2, 0.2),
                "z": (-0.2, 0.2),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        },
    )
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={"position_range": (0.7, 1.3), "velocity_range": (0.0, 0.0)},
    )
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(8.0, 12.0),
        params={"velocity_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3)}},
    )


@configclass
class RewardsCfg:
    """倒地恢复奖励函数。"""

    # 恢复阶段重点奖励姿态回正、速度可控和动作平滑。
    upright = RewTerm(func=mdp.flat_orientation_l2, weight=-12.0)
    base_tilt_l2 = RewTerm(func=mdp.base_tilt_l2, weight=-8.0)
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-1.0)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.1)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.02)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-5.0e-7)
    energy = RewTerm(func=mdp.energy, weight=-0.01)
    arm_joint_vel_l2 = RewTerm(
        func=mdp.joint_vel_l2_selected,
        weight=-0.01,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Arm.*", "Claw"])},
    )
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-10.0)


@configclass
class TerminationsCfg:
    """倒地恢复终止条件。"""

    # 恢复任务允许身体接触地面，因此只保留超时结束。
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class ArmFreeRecoveryEnvCfg(ManagerBasedRLEnvCfg):
    """释放机械臂后的倒地恢复训练配置。"""

    scene: ArmFreeRecoverySceneCfg = ArmFreeRecoverySceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    commands: CommandsCfg = CommandsCfg()

    def __post_init__(self) -> None:
        self.decimation = 4
        self.episode_length_s = 12.0
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15
        self.scene.contact_forces.update_period = self.sim.dt
        self.scene.terrain.terrain_generator.curriculum = True


@configclass
class ArmFreeRecoveryEnvCfg_PLAY(ArmFreeRecoveryEnvCfg):
    """倒地恢复测试配置。"""

    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 20
        self.scene.env_spacing = 2.5
        self.episode_length_s = 20.0
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator.num_rows = 5
        self.scene.terrain.terrain_generator.num_cols = 5
        self.scene.terrain.terrain_generator.curriculum = False
        self.observations.policy.enable_corruption = False
        self.events.push_robot = None
