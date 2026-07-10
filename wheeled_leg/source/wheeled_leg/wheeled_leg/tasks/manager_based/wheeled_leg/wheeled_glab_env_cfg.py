# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
from __future__ import annotations
'''
from isaacsim import SimulationApp

# 启动仿真应用
simulation_app = SimulationApp({"headless": False})

# 启用 ROS2 Bridge 扩展
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.benchmark.services")

enable_extension("isaacsim.ros2_bridge_wheeled")

# 现在你可以安全地 import rclpy 或其他 ROS2 相关的包了
import rclpy
from rclpy.node import Node
/home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg/tools/plot_joint_torque.py
'''
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
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

from .asset.wheeled import WHEELED_CFG  
from .asset.wheeled_arm import WHEELED_ARM_CFG
from .terrains.terrains import ROUGH_TERRAINS_CFG

from . import mdp

##
# Scene definition
##

@configclass
class WheeledGlabSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""
    terrain = TerrainImporterCfg(
        prim_path="/World/terrain",
        terrain_type="generator",
        terrain_generator=ROUGH_TERRAINS_CFG,
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
   
    # robot
    robot: ArticulationCfg = WHEELED_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    target_visual = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Target",
        spawn=sim_utils.SphereCfg(
            radius=0.02, 
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)) # 红色目标点
        ),
    )
    
    #sensor
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*",
        debug_vis=False,
        history_length=3, 
        track_air_time=True
        )


    # lights
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
class CommandsCfg:
    """Command specifications for the MDP."""

    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(8.0, 12.0),
        rel_standing_envs=0.10,
        rel_heading_envs=0.0,
        heading_command=False,
        heading_control_stiffness=0.5,
        debug_vis=False,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 0.6), 
            lin_vel_y=(0.0, 0.0), 
            ang_vel_z=(-0.3, 0.3), 
            heading=(0.0, 0.0)
        )
    )

    ee_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name="claw_Link",
        resampling_time_range=(4.0, 6.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            pos_x=(0.32, 0.55), 
            pos_y=(-0.12, 0.12), 
            pos_z=(0.16, 0.34),
            roll=(0.0, 0.0),
            pitch=(math.pi * 0.5, math.pi * 0.5),
            yaw=(-0.25, 0.25),
        ),
        
    )

@configclass
class ActionsCfg:
    """Action specifications for the MDP."""
    
    joint_pos_leg = mdp.JointPositionActionCfg(
        asset_name="robot", 
        joint_names=["Hip.*", "Knee.*"], 
        scale=0.5, 
        use_default_offset=True
    )
    joint_vel = mdp.JointVelocityActionCfg(
        asset_name="robot", 
        joint_names=["Hub.*"], 
        scale=10.0, 
        use_default_offset=True
    )
    joint_pos_arm = mdp.JointPositionActionCfg(
        asset_name="robot", 
        joint_names=["Arm.*", "Claw"], 
        scale=0.30, 
        use_default_offset=True
    )

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel, 
            noise=Unoise(n_min=-0.2, n_max=0.2)
        )
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel,
            noise=Unoise(n_min=-0.1, n_max=0.1),
        )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        velocity_commands = ObsTerm(
            func=mdp.generated_commands, 
            params={"command_name": "base_velocity"}
        )
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel, 
            noise=Unoise(n_min=-0.01, n_max=0.01)
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel, 
            noise=Unoise(n_min=-0.1, n_max=0.1)
        )
        actions = ObsTerm(
            func=mdp.last_action
        )
        ee_position_to_target = ObsTerm(
            func=mdp.ee_position_error_in_robot_frame,
            params={"asset_cfg": SceneEntityCfg("robot", body_names=["claw_Link"]), "command_name": "ee_pose"}
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
            self.history_length = 3
            self.flatten_history_dim = True
        
    @configclass
    class CriticCfg(ObsGroup):  
        """Observations for critical group."""
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel
        )
        body_contact = ObsTerm(
            func=mdp.body_contact, 
            params={"sensor_cfg": SceneEntityCfg("contact_forces")}
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()

@configclass
class EventCfg:
    """Configuration for events."""

    # startup
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

    # reset
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
        params={
            "position_range": (0.9, 1.1),
            "velocity_range": (0.0, 0.0),
        },
    )

    # interval
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(18.0, 24.0),
        params={"velocity_range": {"x": (-0.15, 0.15), "y": (-0.15, 0.15)}},
    )

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # -- task
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp, 
        weight=8.0, 
        params={"command_name": "base_velocity", "std": 0.75}
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp, 
        weight=1.0, 
        params={"command_name": "base_velocity", "std": 0.75}
    )
    termination_penalty = RewTerm(
        func=mdp.is_terminated, 
        weight=-40.0
    )
    lin_vel_z_l2 = RewTerm(
        func=mdp.lin_vel_z_l2, 
        weight=-1.0
    )
    ang_vel_xy_l2 = RewTerm(
        func=mdp.ang_vel_xy_l2, 
        weight=-0.15
        )
    energy = RewTerm(
        func=mdp.energy, 
        weight=-0.03
    )
    dof_acc_l2 = RewTerm(
        func=mdp.joint_acc_l2, 
        weight=-5.0e-7
    )
    action_rate_l2 = RewTerm(
        func=mdp.action_rate_l2, 
        weight=-0.015
    )
    arm_joint_vel_l2 = RewTerm(
        func=mdp.joint_vel_l2_selected,
        weight=-0.01,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Arm.*", "Claw"])},
    )
    flat_orientation_l2 = RewTerm(
        func=mdp.flat_orientation_l2, 
        weight=-3.0
    )
    base_tilt_l2 = RewTerm(
        func=mdp.base_tilt_l2,
        weight=-4.0,
    )
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-10.0,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", body_names=["base_link", ".*hip.*", ".*knee.*", ".*arm.*", ".*claw.*"]
            ),
            "threshold": 5.0,
        },
    )
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits, 
        weight=-1.0
    )
    
    joint_deviation_legs = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Hip.*", "Knee.*"])},
    )
    leg_symmetry = RewTerm(
        func=mdp.mirrored_joint_deviation_l1,
        weight=-3.0,
        params={
            "left_asset_cfg": SceneEntityCfg("robot", joint_names=["HipL", "KneeL"]),
            "right_asset_cfg": SceneEntityCfg("robot", joint_names=["HipR", "KneeR"]),
        },
    )
    joint_deviation_arms = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.3,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Arm.*", "Claw"])},
    )

    approach_ee_target_exp = RewTerm(
        func=mdp.track_ee_pose_exp, 
        weight=40.0, 
        params={"command_name": "ee_pose", "std": 0.35, "asset_cfg": SceneEntityCfg("robot", body_names=["claw_Link"])}
    )
    approach_ee_target_tanh = RewTerm(
        func=mdp.track_ee_position_tanh,
        weight=60.0,
        params={"command_name": "ee_pose", "std": 0.20, "asset_cfg": SceneEntityCfg("robot", body_names=["claw_Link"])},
    )

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(
        func=mdp.time_out, 
        time_out=True
    )
    
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", body_names=["base_link", ".*hip.*", ".*knee.*", ".*arm.*", ".*claw.*"]
            ),
            "threshold": 5.0,
        },
    )
    
##
# Environment configuration
##


@configclass
class WheeledGlabEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: WheeledGlabSceneCfg = WheeledGlabSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    commands: CommandsCfg = CommandsCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 4
        self.episode_length_s = 20.0

        self.sim.dt = 1/120
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15

        self.scene.contact_forces.update_period = self.sim.dt
        self.scene.terrain.terrain_generator.curriculum = True
