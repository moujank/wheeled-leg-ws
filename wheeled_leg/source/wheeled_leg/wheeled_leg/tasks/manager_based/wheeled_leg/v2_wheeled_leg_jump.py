# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Independent v2 task: stationary jumps, then moving jumps over a bar to a goal."""

from __future__ import annotations

import math
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
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

from . import mdp
from .asset.wheeled_leg_v2_agile import WHEELED_LEG_V2_JUMP_CFG


##
# Scene definition
##
@configclass
class WheeledLegJumpSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path='/World/terrain', terrain_type='plane', collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode='multiply', restitution_combine_mode='multiply',
            static_friction=1.0, dynamic_friction=1.0,
        ),
        debug_vis=False,
    )
    robot: ArticulationCfg = WHEELED_LEG_V2_JUMP_CFG.replace(prim_path='{ENV_REGEX_NS}/Robot')
    # A long transverse cuboid blocks the entire valid lane. The command term
    # changes exposed height at reset by moving this kinematic cuboid vertically.
    jump_barrier: RigidObjectCfg = RigidObjectCfg(
        prim_path='{ENV_REGEX_NS}/JumpBarrier',
        spawn=sim_utils.CuboidCfg(
            size=(0.10, 3.0, 0.20),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            activate_contact_sensors=True,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.35, 0.1)),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.8, dynamic_friction=0.7, restitution=0.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(1.3, 0.0, -0.50)),
    )
    contact_forces = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Robot/.*', history_length=3, track_air_time=True, debug_vis=False,
    )
    sky_light = AssetBaseCfg(
        prim_path='/World/skyLight', spawn=sim_utils.DomeLightCfg(intensity=750.0),
    )


##
# MDP settings
##
@configclass
class CommandsCfg:
    base_velocity = mdp.JumpCourseCommandCfg(
        resampling_time_range=(1000.0, 1000.0), initial_stage=0, debug_vis=False,
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
        # Preserve the baseline 28-value prefix for optional weight transfer; jump state is appended.
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
        jump_course = ObsTerm(func=mdp.jump_course_observation)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ProprioceptionCfg):
        body_contact = ObsTerm(func=mdp.body_contact, params={'sensor_cfg': SceneEntityCfg('contact_forces')})
        jump_course = ObsTerm(func=mdp.jump_course_observation)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class EventCfg:
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
    # Dense launch/flight signals teach the first jump before sparse success is possible.
    jump_height = RewTerm(func=mdp.jump_height_tracking, weight=3.0)
    jump_flight = RewTerm(func=mdp.jump_flight_reward, weight=8.0)
    launch_velocity = RewTerm(func=mdp.jump_launch_velocity, weight=3.0)
    jump_landing = RewTerm(func=mdp.jump_landing_reward, weight=600.0)
    goal_success = RewTerm(func=mdp.jump_success_reward, weight=1500.0)
    goal_progress = RewTerm(func=mdp.jump_goal_progress, weight=4.0)
    track_lin_vel_xy_exp = RewTerm(func=mdp.track_lin_vel_xy_exp, weight=0.3,
                                 params={'command_name': 'base_velocity', 'std': 0.6})
    stationary_drift = RewTerm(func=mdp.jump_stationary_drift, weight=-8.0)
    recovery_vertical_velocity = RewTerm(func=mdp.jump_recovery_velocity_l2, weight=-0.3)
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-150.0)
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-1.0)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.1)
    energy = RewTerm(func=mdp.energy, weight=-1.0e-5)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.003)
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-1.0)
    # Bilateral coordination: jump gates preserve legitimate task-specific motion.
    mirrored_legs = RewTerm(
        func=mdp.mirrored_leg_position_l2, weight=-0.3,
        params={
            "left_asset_cfg": SceneEntityCfg("robot", joint_names=["HipL", "KneeL"], preserve_order=True),
            "right_asset_cfg": SceneEntityCfg("robot", joint_names=["HipR", "KneeR"], preserve_order=True),
            "deadband": 0.05,
            "mode": "jump",
        },
    )
    mirrored_leg_velocity = RewTerm(
        func=mdp.mirrored_leg_velocity_l2, weight=-0.008,
        params={
            "left_asset_cfg": SceneEntityCfg("robot", joint_names=["HipL", "KneeL"], preserve_order=True),
            "right_asset_cfg": SceneEntityCfg("robot", joint_names=["HipR", "KneeR"], preserve_order=True),
            "mode": "jump",
        },
    )
    mirrored_leg_action = RewTerm(
        func=mdp.mirrored_leg_action_l2, weight=-0.04,
        params={
            "mode": "jump",
        },
    )
    mirrored_leg_action_rate = RewTerm(
        func=mdp.mirrored_leg_action_rate_l2, weight=-0.02,
        params={
            "mode": "jump",
        },
    )
    straight_wheel_balance = RewTerm(
        func=mdp.straight_wheel_speed_balance, weight=-0.001,
        params={
            "wheel_asset_cfg": SceneEntityCfg("robot", joint_names=["HubL", "HubR"], preserve_order=True),
            "mode": "jump",
        },
    )
    wheel_slip = RewTerm(
        func=mdp.wheel_slip_l2, weight=-0.08,
        params={
            "wheel_asset_cfg": SceneEntityCfg("robot", body_names=["HubL", "HubR"], preserve_order=True),
            "wheel_sensor_cfg": SceneEntityCfg("contact_forces", body_names=["HubL", "HubR"], preserve_order=True),
            "mode": "jump",
        },
    )
    single_side_support = RewTerm(
        func=mdp.single_side_support_penalty, weight=-0.2,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["HubL", "HubR"], preserve_order=True),
            "unilateral_grace_s": 0.2,
            "mode": "jump",
        },
    )
    straight_yaw_drift = RewTerm(
        func=mdp.straight_yaw_drift_l2, weight=-0.12,
    )


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    success = DoneTerm(func=mdp.jump_success, time_out=True)
    body_contact = DoneTerm(func=mdp.illegal_contact,
                           params={'sensor_cfg': SceneEntityCfg('contact_forces', body_names=['base_link', 'Hip.*', 'Knee.*']), 'threshold': 1.0})
    excessive_tilt = DoneTerm(func=mdp.bad_orientation, params={'limit_angle': math.radians(60.0)})
    out_of_lane = DoneTerm(func=mdp.jump_course_out_of_bounds)
    unqualified_crossing = DoneTerm(func=mdp.jump_failed_crossing)


@configclass
class CurriculumCfg:
    jump_stage = CurrTerm(func=mdp.jump_course_curriculum,
                         params={'min_episodes': 20, 'success_threshold': 0.7})


##
# Environment configuration
##
@configclass
class WheeledLegJumpEnvCfg(ManagerBasedRLEnvCfg):
    scene: WheeledLegJumpSceneCfg = WheeledLegJumpSceneCfg(num_envs=4096, env_spacing=5.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    commands: CommandsCfg = CommandsCfg()
    curriculum: CurriculumCfg = CurriculumCfg()
    warm_start_appended_observations: int = 12
    warm_start_noise_std: float = 0.6

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 12.0
        self.sim.dt = 0.002
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15
        self.scene.contact_forces.update_period = self.sim.dt
        cfg = self.commands.base_velocity
        if not 0 <= cfg.initial_stage <= 8:
            raise ValueError('Jump initial_stage must be in [0, 8].')
        if len(cfg.stationary_heights) != 3 or len(cfg.barrier_heights) != 5:
            raise ValueError('Jump course requires three stationary and five obstacle heights.')
        if self.scene.jump_barrier.spawn.size[0] != cfg.barrier_width:
            raise ValueError('The jump command width must match the physical barrier.')
        if not (0 < cfg.crouch_s < cfg.takeoff_s < cfg.stationary_period_s):
            raise ValueError('The jump preparation and takeoff must fit in the stationary period.')
        if not (0 < cfg.goal_tolerance < cfg.goal_x - cfg.barrier_x):
            raise ValueError('The goal must lie beyond the obstacle.')
        if not (0 < cfg.approach_speed and cfg.approach_speed * cfg.crouch_s + cfg.barrier_width / 2
                + cfg.wheel_radius < cfg.launch_distance < cfg.barrier_x):
            raise ValueError('Launch distance must leave room to crouch before the obstacle.')
        if max(cfg.barrier_heights) > self.scene.jump_barrier.spawn.size[2]:
            raise ValueError('Exposed obstacle heights must fit inside the cuboid.')
        if self.scene.jump_barrier.spawn.size[1] / 2 <= cfg.lane_half_width + cfg.wheel_radius:
            raise ValueError('The obstacle must span the entire valid jumping lane.')


@configclass
class WheeledLegJumpEnvCfg_PLAY(WheeledLegJumpEnvCfg):
    torque_logging_enabled: bool = True
    torque_log_folder_name: str = 'play_joint_torque'
    torque_log_env_index: int = 0
    torque_log_flush_interval: int = 50

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 20
        self.commands.base_velocity.initial_stage = 8
        self.commands.base_velocity.debug_vis = True
        self.curriculum.jump_stage = None
        self.observations.policy.enable_corruption = False
        self.events.physics_material = None
        self.events.reset_base.params['pose_range'] = {}
        self.events.reset_robot_joints.params['position_range'] = (1.0, 1.0)
