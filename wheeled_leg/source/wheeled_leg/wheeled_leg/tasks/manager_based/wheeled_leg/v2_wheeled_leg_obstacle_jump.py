# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Active stair traversal with terrain-aware stability and staged robustness."""

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
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from . import mdp
from .asset.wheeled_leg_v2_agile import WHEELED_LEG_V2_OBSTACLE_CFG
from .terrains.obstacle_jump import OBSTACLE_JUMP_TERRAINS_CFG


##
# Scene definition
##
@configclass
class WheeledLegObstacleJumpSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path='/World/terrain', terrain_type='generator',
        terrain_generator=OBSTACLE_JUMP_TERRAINS_CFG.copy(), max_init_terrain_level=0,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode='multiply', restitution_combine_mode='multiply',
            static_friction=1.0, dynamic_friction=1.0,
        ),
        debug_vis=False,
    )
    robot: ArticulationCfg = WHEELED_LEG_V2_OBSTACLE_CFG.replace(prim_path='{ENV_REGEX_NS}/Robot')
    contact_forces = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Robot/.*', history_length=3, track_air_time=True, debug_vis=False,
    )
    height_scanner = RayCasterCfg(
        prim_path='{ENV_REGEX_NS}/Robot/base_link',
        offset=RayCasterCfg.OffsetCfg(pos=(0.5, 0.0, 2.0)), ray_alignment='yaw',
        pattern_cfg=patterns.GridPatternCfg(resolution=0.2, size=(1.6, 1.0)),
        mesh_prim_paths=['/World/terrain'], debug_vis=False,
    )
    sky_light = AssetBaseCfg(
        prim_path='/World/skyLight', spawn=sim_utils.DomeLightCfg(intensity=750.0),
    )


##
# MDP settings
##
@configclass
class CommandsCfg:
    base_velocity = mdp.StableTraversalVelocityCommandCfg(
        asset_name='robot', resampling_time_range=(2.5, 4.0),
        rel_standing_envs=0.15, rel_heading_envs=0.0, heading_command=False, debug_vis=False,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.75, 1.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(-0.8, 0.8),
        ),
    )


@configclass
class ActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(
        asset_name='robot', joint_names=['Hip.*', 'Knee.*'], scale=0.65,
        clip={'HipL': (0.05, 1.45), 'KneeL': (0.10, 2.60),
              'HipR': (-1.45, -0.05), 'KneeR': (-2.60, -0.10)},
        use_default_offset=True,
    )
    joint_vel = mdp.JointVelocityActionCfg(
        asset_name='robot', joint_names=['Hub.*'], scale=5.0,
        clip={'Hub.*': (-12.0, 12.0)}, use_default_offset=True,
    )


@configclass
class ObservationsCfg:
    @configclass
    class ProprioceptionCfg(ObsGroup):
        # Preserve the baseline 28-value prefix for optional weight transfer.
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, clip=(-5.0, 5.0), noise=Unoise(n_min=-0.03, n_max=0.03))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, scale=0.25, clip=(-10.0, 10.0), noise=Unoise(n_min=-0.05, n_max=0.05))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.02, n_max=0.02))
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={'command_name': 'base_velocity'})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel,
                            params={'asset_cfg': SceneEntityCfg('robot', joint_names=['Hip.*', 'Knee.*'])},
                            noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, scale=0.1, clip=(-30.0, 30.0), noise=Unoise(n_min=-0.05, n_max=0.05))
        actions = ObsTerm(func=mdp.last_action, clip=(-5.0, 5.0))

    @configclass
    class PolicyCfg(ProprioceptionCfg):
        terrain_heights = ObsTerm(func=mdp.finite_height_scan)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ProprioceptionCfg):
        body_contact = ObsTerm(func=mdp.body_contact, params={'sensor_cfg': SceneEntityCfg('contact_forces')})
        terrain_heights = ObsTerm(func=mdp.finite_height_scan)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class EventCfg:
    body_push = EventTerm(
        func=mdp.RandomBodyForcePulse, mode='interval',
        interval_range_s=(0.008, 0.008), is_global_time=True,
        params={'interval_s': (4.0, 7.0), 'duration_s': (0.08, 0.20),
                'acceleration_range': (0.5, 3.0), 'delay_steps': 40000,
                'ramp_steps': 80000, 'grace_s': 2.0},
    )
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material, mode='startup',
        params={'asset_cfg': SceneEntityCfg('robot', body_names='.*'),
                'static_friction_range': (0.8, 1.1), 'dynamic_friction_range': (0.7, 0.9),
                'restitution_range': (0.0, 0.0), 'num_buckets': 64},
    )
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform, mode='reset',
        params={'pose_range': {'x': (-0.03, 0.03), 'y': (-0.03, 0.03), 'yaw': (-0.05, 0.05)},
                'velocity_range': {}},
    )
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale, mode='reset',
        params={'position_range': (0.98, 1.02), 'velocity_range': (0.0, 0.0)},
    )


@configclass
class RewardsCfg:
    forward_progress = RewTerm(func=mdp.stable_traversal_progress, weight=4.0)
    supported_ascent = RewTerm(func=mdp.SupportedAscent, weight=4.0)
    commanded_stall = RewTerm(func=mdp.commanded_stall, weight=-1.0)
    traversal_success = RewTerm(func=mdp.stable_waypoint_reward, weight=150.0)
    track_lin_vel_xy_exp = RewTerm(func=mdp.track_lin_vel_xy_exp, weight=4.5,
                                 params={'command_name': 'base_velocity', 'std': 0.5})
    track_ang_vel_z_exp = RewTerm(func=mdp.track_ang_vel_z_exp, weight=1.0,
                                params={'command_name': 'base_velocity', 'std': 0.5})
    lateral_deviation = RewTerm(func=mdp.traversal_lateral_deviation, weight=-0.5)
    flat_vertical_velocity = RewTerm(func=mdp.traversal_flat_vertical_velocity, weight=-0.5)
    grounded_vertical_velocity = RewTerm(func=mdp.grounded_vertical_motion, weight=-1.0)
    stop_motion = RewTerm(func=mdp.stop_motion, weight=-2.0)
    excess_forward_speed = RewTerm(func=mdp.excess_forward_speed, weight=-0.75)
    flat_leg_motion = RewTerm(func=mdp.traversal_flat_leg_motion, weight=-0.01,
                             params={'asset_cfg': SceneEntityCfg('robot', joint_names=['Hip.*', 'Knee.*'])})
    alive = RewTerm(func=mdp.is_alive, weight=0.2)
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-100.0)
    ang_vel_xy_l2 = RewTerm(func=mdp.terrain_stability_ang_vel, weight=-0.2)
    flat_orientation_l2 = RewTerm(func=mdp.terrain_stability_orientation, weight=-3.0)
    energy = RewTerm(func=mdp.energy, weight=-1.0e-5)
    action_rate_l2 = RewTerm(func=mdp.terrain_action_rate, weight=-0.015)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-2.0e-8)
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-1.0)
    # Local upward edges permit deliberate leg/pitch motion. Recovery, roll,
    # wheel smoothing and stopping remain constrained.
    # Bilateral coordination: obstacle gates preserve legitimate task-specific motion.
    mirrored_legs = RewTerm(
        func=mdp.mirrored_leg_position_l2, weight=-0.15,
        params={
            "left_asset_cfg": SceneEntityCfg("robot", joint_names=["HipL", "KneeL"], preserve_order=True),
            "right_asset_cfg": SceneEntityCfg("robot", joint_names=["HipR", "KneeR"], preserve_order=True),
            "deadband": 0.1,
            "mode": "obstacle",
        },
    )
    mirrored_leg_velocity = RewTerm(
        func=mdp.mirrored_leg_velocity_l2, weight=-0.004,
        params={
            "left_asset_cfg": SceneEntityCfg("robot", joint_names=["HipL", "KneeL"], preserve_order=True),
            "right_asset_cfg": SceneEntityCfg("robot", joint_names=["HipR", "KneeR"], preserve_order=True),
            "mode": "obstacle",
        },
    )
    mirrored_leg_action = RewTerm(
        func=mdp.mirrored_leg_action_l2, weight=-0.025,
        params={
            "mode": "obstacle",
        },
    )
    mirrored_leg_action_rate = RewTerm(
        func=mdp.mirrored_leg_action_rate_l2, weight=-0.01,
        params={
            "mode": "obstacle",
        },
    )
    straight_wheel_balance = RewTerm(
        func=mdp.straight_wheel_speed_balance, weight=-0.001,
        params={
            "wheel_asset_cfg": SceneEntityCfg("robot", joint_names=["HubL", "HubR"], preserve_order=True),
            "mode": "obstacle",
        },
    )
    wheel_slip = RewTerm(
        func=mdp.wheel_slip_l2, weight=-0.06,
        params={
            "wheel_asset_cfg": SceneEntityCfg("robot", body_names=["HubL", "HubR"], preserve_order=True),
            "wheel_sensor_cfg": SceneEntityCfg("contact_forces", body_names=["HubL", "HubR"], preserve_order=True),
            "mode": "obstacle",
        },
    )
    single_side_support = RewTerm(
        func=mdp.single_side_support_penalty, weight=-0.12,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["HubL", "HubR"], preserve_order=True),
            "unilateral_grace_s": 0.3,
            "mode": "obstacle",
        },
    )
    straight_yaw_drift = RewTerm(
        func=mdp.straight_yaw_drift_l2, weight=-0.08,
    )


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(func=mdp.illegal_contact,
                            params={'sensor_cfg': SceneEntityCfg('contact_forces', body_names=['base_link']), 'threshold': 1.0})
    excessive_tilt = DoneTerm(func=mdp.bad_orientation, params={'limit_angle': math.radians(60.0)})
    terrain_boundary = DoneTerm(func=mdp.terrain_patch_boundary, params={'margin': 0.25})


@configclass
class CurriculumCfg:
    # Outcome counts must refer to the terrain just attempted, before promotion.
    traversal_outcomes = CurrTerm(func=mdp.traversal_outcomes,
                                 params={'success_command_name': 'base_velocity', 'success_attribute': 'terrain_success'})
    terrain_levels = CurrTerm(func=mdp.traversal_curriculum,
                             params={'success_command_name': 'base_velocity', 'success_attribute': 'terrain_success'})
    terrain_distribution = CurrTerm(func=mdp.terrain_level_distribution)


##
# Environment configuration
##
@configclass
class WheeledLegObstacleJumpEnvCfg(ManagerBasedRLEnvCfg):
    """Thirty-second patrols; waypoints never reset the robot or end an episode."""
    scene: WheeledLegObstacleJumpSceneCfg = WheeledLegObstacleJumpSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    commands: CommandsCfg = CommandsCfg()
    curriculum: CurriculumCfg = CurriculumCfg()
    warm_start_appended_observations: int = 54
    warm_start_noise_std: float = 0.6

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 30.0
        self.sim.dt = 0.002
        if self.events.body_push is not None:
            self.events.body_push.interval_range_s = (self.decimation * self.sim.dt,) * 2
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15
        self.scene.contact_forces.update_period = self.sim.dt
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt


@configclass
class WheeledLegObstacleJumpEnvCfg_PLAY(WheeledLegObstacleJumpEnvCfg):
    torque_logging_enabled: bool = True
    torque_log_folder_name: str = 'play_joint_torque'
    torque_log_env_index: int = 0
    torque_log_flush_interval: int = 50

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 20
        # Evaluate only full 20 cm geometry, not a random selection of easier rows.
        self.scene.terrain.terrain_generator.num_rows = 1
        self.scene.terrain.terrain_generator.difficulty_range = (1.0, 1.0)
        self.scene.terrain.max_init_terrain_level = 0
        self.curriculum.terrain_levels = None
        self.curriculum.terrain_distribution = None
        self.observations.policy.enable_corruption = False
        self.events.physics_material = None
        self.events.reset_base.params['pose_range'] = {}
        self.events.reset_robot_joints.params['position_range'] = (1.0, 1.0)
        # Keep dynamic commands and pushes in PLAY for robustness evaluation.
        # A fresh PLAY process otherwise starts at zero push curriculum strength.
        self.events.body_push.params['ramp_steps'] = 1
        self.events.body_push.params['delay_steps'] = 0
        self.commands.base_velocity.robustness_delay_steps = 0
        self.commands.base_velocity.robustness_ramp_steps = 1
