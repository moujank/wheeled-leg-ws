# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
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
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

from .asset.wheeled_leg_v2 import WHEELED_CFG  
from .terrains.terrains import GRAVEL_TERRAINS_CFG, ROUGH_TERRAINS_CFG, PYRAMID_TERRAINS_CFG

from . import mdp

##
# Scene definition
##

@configclass
class WheeledLegSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""
    terrain = TerrainImporterCfg(
        prim_path="/World/terrain",
        terrain_type="generator",
        terrain_generator=GRAVEL_TERRAINS_CFG,
        max_init_terrain_level=5,
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
    robot: ArticulationCfg = WHEELED_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

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
            heading=(-math.pi, math.pi)
        )
    )

@configclass
class ActionsCfg:
    """Action specifications for the MDP."""
    
    joint_pos = mdp.JointPositionActionCfg(
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
        
        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
            # self.history_length = 10
            # self.flatten_history_dim = False

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
            "mass_distribution_params": (0.0, 5.0),
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

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (-0.5, 0.5),
            "velocity_range": (0.0, 0.0),
        },
    )

    # interval
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    )

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # -- task
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp, 
        weight=50.0, 
        params={"command_name": "base_velocity", "std": 0.5}
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp, 
        weight=0.5, 
        params={"command_name": "base_velocity", "std": 0.5}
    )
    # -- penalties
    termination_penalty = RewTerm(
        func=mdp.is_terminated, 
        weight=-200.0
    )
    lin_vel_z_l2 = RewTerm(
        func=mdp.lin_vel_z_l2, 
        weight=-2.0
    )
    ang_vel_xy_l2 = RewTerm(
        func=mdp.ang_vel_xy_l2, 
        weight=-0.05
        )
    energy = RewTerm(
        func=mdp.energy, 
        weight=-1e-3
    )
    dof_acc_l2 = RewTerm(
        func=mdp.joint_acc_l2, 
        weight=-2.5e-7
    )
    action_rate_l2 = RewTerm(
        func=mdp.action_rate_l2, 
        weight=-0.01
    )
    flat_orientation_l2 = RewTerm(
        func=mdp.flat_orientation_l2, 
        weight=-20.0
    )
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=["Hip.*", "Knee.*"]), "threshold": 1.0},
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
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["base_link", "Hip.*", "Knee.*"]), "threshold": 1.0
        },
    )


##
# Environment configuration
##


@configclass
class WheeledLegEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: WheeledLegSceneCfg = WheeledLegSceneCfg(num_envs=4096, env_spacing=2.5)
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

        # 与基础底盘任务保持一致：物理 500 Hz，动作更新 125 Hz。
        self.sim.dt = 0.002
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15

        self.scene.contact_forces.update_period = self.sim.dt
        self.scene.terrain.terrain_generator.curriculum = False

@configclass
class WheeledLegEnvCfg_PLAY(WheeledLegEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
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
