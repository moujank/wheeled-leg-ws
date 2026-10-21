# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""Basic multi-terrain stable locomotion task, configured independently of versioned tasks."""

from __future__ import annotations

import math

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
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
from .terrains.terrain_balance import TERRAIN_BALANCE_TERRAINS_CFG

from . import mdp

##
# Scene definition
##

@configclass
class WheeledLegTerrainBalanceSceneCfg(InteractiveSceneCfg):
    """Basic multi-terrain balance scene using the wheeled_leg_v2 asset."""
    terrain = TerrainImporterCfg(
        prim_path="/World/terrain",
        # Every terrain type starts at its easiest row.
        terrain_type="generator",
        terrain_generator=TERRAIN_BALANCE_TERRAINS_CFG.copy(),
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
    robot: ArticulationCfg = WHEELED_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        # A mirrored knee angle of twice the hip angle brings the wheels under the base.
        # URDF + wheel mesh gives ~0.550 m grounded height for this nominal pose.
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
        # More leg damping reduces ringing without increasing stiffness or wheel drag.
        # Implicit actuators ignore the legacy velocity_limit field in this Isaac Lab version.
        actuators={
            name: actuator.replace(
                velocity_limit=None,
                velocity_limit_sim=2.0 * math.pi,
                damping=2.5 if name in ("hip_act", "knee_act") else actuator.damping,
            )
            for name, actuator in WHEELED_CFG.actuators.items()
        },
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
        # Keep the command through a training episode to encourage terrain traversal.
        resampling_time_range=(20.0, 20.0),
        rel_standing_envs=0.2,
        rel_heading_envs=0.0,
        heading_command=False,
        heading_control_stiffness=0.5,
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.45, 0.45),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(-0.25, 0.25),
            heading=None
        )
    )

@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["Hip.*", "Knee.*"],
        scale=0.25,
        # Bounds apply to the final joint targets, after scale and default offset.
        clip={
            "HipL": (0.2, 1.1), "KneeL": (0.8, 1.8),
            "HipR": (-1.1, -0.2), "KneeR": (-1.8, -0.8),
        },
        use_default_offset=True
    )
    joint_vel = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=["Hub.*"],
        scale=4.0,
        # Stay below the asset actuator velocity limit (2*pi rad/s).
        clip={"Hub.*": (-6.0, 6.0)},
        use_default_offset=True
    )

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # Give the policy velocity feedback; wheel positions are unbounded and omitted.
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel,
            noise=Unoise(n_min=-0.03, n_max=0.03),
            clip=(-5.0, 5.0),
        )
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            noise=Unoise(n_min=-0.05, n_max=0.05),
            scale=0.25,
            clip=(-10.0, 10.0),
        )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.02, n_max=0.02),
        )
        velocity_commands = ObsTerm(
            func=mdp.generated_commands,
            params={"command_name": "base_velocity"},
        )
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Hip.*", "Knee.*"])},
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            noise=Unoise(n_min=-0.05, n_max=0.05),
            scale=0.1,
            clip=(-30.0, 30.0),
        )
        actions = ObsTerm(func=mdp.last_action, clip=(-5.0, 5.0))

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(PolicyCfg):
        """The critic sees the full policy state and privileged contact information."""

        body_contact = ObsTerm(
            func=mdp.body_contact,
            params={"sensor_cfg": SceneEntityCfg("contact_forces")},
        )

        def __post_init__(self):
            super().__post_init__()
            self.enable_corruption = False

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
            "static_friction_range": (0.8, 1.0),
            "dynamic_friction_range": (0.7, 0.8),
            "restitution_range": (0.0, 0.005),
            "num_buckets": 64,
        },
    )

    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "mass_distribution_params": (0.0, 0.5),
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
            "pose_range": {"x": (-0.05, 0.05), "y": (-0.05, 0.05), "yaw": (-0.1, 0.1)},
            "velocity_range": {
                "x": (-0.05, 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.05, 0.05),
                "roll": (-0.05, 0.05),
                "pitch": (-0.05, 0.05),
                "yaw": (-0.05, 0.05),
            },
        },
    )

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            # Scale around the default pose while preserving left/right joint signs.
            "position_range": (0.95, 1.05),
            "velocity_range": (0.0, 0.0),
        },
    )

    # Add pushes after the basic balance policy is reliable.
    push_robot = None

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # -- task
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp,
        weight=3.0,
        # Distinguish stationary balance from tracking the relatively small speed commands.
        params={"command_name": "base_velocity", "std": 0.25}
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": 0.5}
    )
    # -- balance
    alive = RewTerm(func=mdp.is_alive, weight=1.0)

    # -- penalties
    termination_penalty = RewTerm(
        func=mdp.is_terminated,
        weight=-50.0
    )
    lin_vel_z_l2 = RewTerm(
        func=mdp.lin_vel_z_l2,
        weight=-6.0
    )
    ang_vel_xy_l2 = RewTerm(
        func=mdp.ang_vel_xy_l2,
        weight=-0.5
        )
    energy = RewTerm(
        func=mdp.energy,
        weight=-2e-4
    )
    dof_acc_l2 = RewTerm(
        func=mdp.joint_acc_l2,
        weight=-1e-7
    )
    action_rate_l2 = RewTerm(
        func=mdp.action_rate_l2,
        weight=-0.03
    )
    flat_orientation_l2 = RewTerm(
        func=mdp.flat_orientation_l2,
        weight=-1.5
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
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Hip.*", "Knee.*"])},
    )

    leg_joint_vel = RewTerm(
        func=mdp.joint_vel_l2_selected,
        weight=-0.02,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Hip.*", "Knee.*"])},
    )
    # Extra smoothness for the leg position commands, without slowing wheel commands.
    leg_action_rate_l2 = RewTerm(
        func=mdp.action_rate_l2_selected,
        weight=-0.07,
        params={"action_name": "joint_pos"},
    )
    leg_joint_acc_l2 = RewTerm(
        func=mdp.joint_acc_l2,
        weight=-5e-7,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Hip.*", "Knee.*"])},
    )
    # Bilateral coordination: terrain gates preserve legitimate task-specific motion.
    mirrored_legs = RewTerm(
        func=mdp.mirrored_leg_position_l2, weight=-0.35,
        params={
            "left_asset_cfg": SceneEntityCfg("robot", joint_names=["HipL", "KneeL"], preserve_order=True),
            "right_asset_cfg": SceneEntityCfg("robot", joint_names=["HipR", "KneeR"], preserve_order=True),
            "deadband": 0.06,
            "mode": "terrain",
        },
    )
    mirrored_leg_velocity = RewTerm(
        func=mdp.mirrored_leg_velocity_l2, weight=-0.009,
        params={
            "left_asset_cfg": SceneEntityCfg("robot", joint_names=["HipL", "KneeL"], preserve_order=True),
            "right_asset_cfg": SceneEntityCfg("robot", joint_names=["HipR", "KneeR"], preserve_order=True),
            "mode": "terrain",
        },
    )
    mirrored_leg_action = RewTerm(
        func=mdp.mirrored_leg_action_l2, weight=-0.05,
        params={
            "mode": "terrain",
        },
    )
    mirrored_leg_action_rate = RewTerm(
        func=mdp.mirrored_leg_action_rate_l2, weight=-0.025,
        params={
            "mode": "terrain",
        },
    )
    straight_wheel_balance = RewTerm(
        func=mdp.straight_wheel_speed_balance, weight=-0.003,
        params={
            "wheel_asset_cfg": SceneEntityCfg("robot", joint_names=["HubL", "HubR"], preserve_order=True),
            "mode": "terrain",
        },
    )
    wheel_slip = RewTerm(
        func=mdp.wheel_slip_l2, weight=-0.1,
        params={
            "wheel_asset_cfg": SceneEntityCfg("robot", body_names=["HubL", "HubR"], preserve_order=True),
            "wheel_sensor_cfg": SceneEntityCfg("contact_forces", body_names=["HubL", "HubR"], preserve_order=True),
            "mode": "terrain",
        },
    )
    single_side_support = RewTerm(
        func=mdp.single_side_support_penalty, weight=-0.22,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["HubL", "HubR"], preserve_order=True),
            "unilateral_grace_s": 0.25,
            "mode": "terrain",
        },
    )
    straight_yaw_drift = RewTerm(
        func=mdp.straight_yaw_drift_l2, weight=-0.1,
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

    # Reset before leaving the assigned terrain patch; this is a time-limit truncation.
    terrain_boundary = DoneTerm(
        func=mdp.terrain_patch_boundary,
        time_out=True,
        params={"margin": 0.25},
    )

    # Flat-orientation penalties alone cannot distinguish upright from inverted states.
    excessive_tilt = DoneTerm(
        func=mdp.bad_orientation,
        params={"limit_angle": math.radians(60.0)},
    )


@configclass
class CurriculumCfg:
    """Increase terrain difficulty only after stable command tracking and traversal."""

    terrain_levels = CurrTerm(
        func=mdp.terrain_levels_stable_tracking,
        params={
            "min_distance": 2.0,
            "min_traversal_distance": 0.75,
            "distance_fraction": 0.6,
            "min_command_speed": 0.1,
            "min_episode_duration": 5.0,
            "tracking_threshold": 0.75,
            "yaw_tracking_threshold": 0.7,
            "command_name": "base_velocity",
        },
    )
    terrain_distribution = CurrTerm(func=mdp.terrain_level_distribution)


##
# Environment configuration
##


@configclass
class WheeledLegTerrainBalanceEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: WheeledLegTerrainBalanceSceneCfg = WheeledLegTerrainBalanceSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    commands: CommandsCfg = CommandsCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

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

@configclass
class WheeledLegTerrainBalanceEnvCfg_PLAY(WheeledLegTerrainBalanceEnvCfg):
    # Consumed by scripts/rsl_rl/play.py; compatible with tools/plot_joint_torque.py.
    torque_logging_enabled: bool = True
    torque_log_folder_name: str = "play_joint_torque"
    torque_log_env_index: int = 0
    torque_log_flush_interval: int = 50

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.episode_length_s = 20.0

        # Sample all terrain rows during evaluation, without changing them after resets.
        self.scene.terrain.max_init_terrain_level = None
        self.curriculum.terrain_levels = None
        self.curriculum.terrain_distribution = None

        self.commands.base_velocity.ranges.lin_vel_x = (0.4, 0.4)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        self.commands.base_velocity.rel_standing_envs = 0.0

        self.observations.policy.enable_corruption = False

        # Evaluate at the nominal model and pose with zero reset velocity.
        self.events.physics_material = None
        self.events.add_base_mass = None
        self.events.reset_base.params["pose_range"] = {}
        self.events.reset_base.params["velocity_range"] = {}
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.base_external_force_torque = None
        self.events.push_robot = None
