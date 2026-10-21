# SPDX-License-Identifier: BSD-3-Clause
"""Long-duration traversal, step velocity commands and finite-duration pushes."""

import math

import torch
from isaaclab.managers import ManagerTermBase
from isaaclab.utils import configclass

from .commands import TraversalVelocityCommand, TraversalVelocityCommandCfg
from .traversal import traversal_flat_weight


def robustness_strength(env, delay_steps, ramp_steps):
    """Global control-frame schedule; a new training process starts it again."""
    return min(1.0, max(0.0, (env.common_step_counter - delay_steps) / max(ramp_steps, 1)))


class StableTraversalVelocityCommand(TraversalVelocityCommand):
    """Shuttle between the origin and a waypoint without resetting the robot.

    Turns use a heading controller, so the forward scanner observes the next
    stretch of terrain. Random commands are steps, including exact zero twists.
    A waypoint pays once, only after a continuous settled stop with both wheels
    grounded. No state/pose is teleported at a waypoint.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.target_x = torch.full((self.num_envs,), cfg.waypoint_x, device=self.device)
        self.direction = torch.ones(self.num_envs, device=self.device)
        self.sampled_speed = torch.zeros(self.num_envs, device=self.device)
        self.sampled_stop = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self.settled_s = torch.zeros(self.num_envs, device=self.device)
        self.waypoints = torch.zeros(self.num_envs, device=self.device)
        self.waypoint_pulse = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self.last_step = torch.full((self.num_envs,), -1, device=self.device, dtype=torch.long)
        for name in ('tilt_deg', 'roll_pitch_speed', 'stop_fraction', 'stop_speed',
                     'stop_position_drift', 'stop_settled_fraction', 'waypoints', 'long_episode_success',
                     'terrain_success', 'climb_fraction', 'moving_stall_fraction'):
            self.metrics[name] = torch.zeros(self.num_envs, device=self.device)
        self.stop_anchor = torch.zeros(self.num_envs, 2, device=self.device)
        self.was_stopping = torch.zeros_like(self.sampled_stop)
        self.stop_samples = torch.zeros(self.num_envs, device=self.device)

    def _ids(self, env_ids):
        return torch.arange(self.num_envs, device=self.device)[slice(None) if env_ids is None else env_ids]

    def reset(self, env_ids=None):
        ids = self._ids(env_ids)
        # Reset logging must use the FINAL outcome. The last normal command
        # update precedes a terminal fall by one step, so its cached flag may
        # still report a successful patrol that subsequently failed.
        self.metrics['long_episode_success'][ids] = self.patrol_success[ids].float()
        self.metrics['terrain_success'][ids] = self.terrain_success[ids].float()
        self.metrics['waypoints'][ids] = self.waypoints[ids]
        # The base reset samples commands. Clear route state BEFORE that call;
        # episode_length_buf still contains the old episode's duration here.
        self.target_x[ids] = self.cfg.waypoint_x
        self.direction[ids] = 1.0
        self.settled_s[ids] = 0.0
        self.waypoints[ids] = 0.0
        self.waypoint_pulse[ids] = False
        self.last_step[ids] = -1
        self.was_stopping[ids] = False
        self.stop_samples[ids] = 0.0
        extras = super().reset(ids)
        self._write_command(ids)
        return extras

    def _resample_command(self, env_ids):
        ids = self._ids(env_ids)
        # Alternating low/high bands makes actual acceleration/deceleration
        # steps frequent rather than resampling almost identical speeds.
        low = torch.empty(len(ids), device=self.device).uniform_(*self.cfg.slow_speed_range)
        high = torch.empty(len(ids), device=self.device).uniform_(*self.cfg.ranges.lin_vel_x)
        self.sampled_speed[ids] = torch.where(self.sampled_speed[ids] > self.cfg.slow_speed_range[1], low, high)
        strength = robustness_strength(self._env, self.cfg.robustness_delay_steps, self.cfg.robustness_ramp_steps)
        self.sampled_stop[ids] = torch.rand(len(ids), device=self.device) < self.cfg.rel_standing_envs * strength

    def update_state(self):
        """Update once per physics-control frame, also when read by rewards/curriculum."""
        step = self._env.episode_length_buf
        fresh = step != self.last_step
        self.waypoint_pulse[fresh] = False
        data = self.robot.data
        x = data.root_pos_w[:, 0] - self._env.scene.env_origins[:, 0]
        y = data.root_pos_w[:, 1] - self._env.scene.env_origins[:, 1]
        near = (self.target_x - x).abs() < self.cfg.waypoint_tolerance
        grounded = (self._env.scene.sensors['contact_forces'].data.current_air_time[:, self._hub_ids] < .016).all(dim=1)
        settled = (near & (y.abs() < .7) & grounded
                   & (data.projected_gravity_b[:, 2] < -math.cos(math.radians(20)))
                   & (torch.linalg.vector_norm(data.root_lin_vel_w, dim=1) < .2)
                   & (torch.linalg.vector_norm(data.root_ang_vel_b, dim=1) < .6))
        self.settled_s[fresh] = torch.where(settled[fresh], self.settled_s[fresh] + self._env.step_dt, 0.0)
        reached = fresh & (self.settled_s >= self.cfg.settle_time_s)
        self.waypoint_pulse[reached] = True
        self.waypoints[reached] += 1
        self.direction[reached] *= -1
        self.target_x[reached] = torch.where(self.direction[reached] > 0, self.cfg.waypoint_x, 0.0)
        self.settled_s[reached] = 0
        self.last_step[fresh] = step[fresh]

    @property
    def terrain_success(self):
        """Reach one settled waypoint and survive the episode; turning is separate."""
        self.update_state()
        return (self.waypoints >= 1) & ~self._env.termination_manager.terminated

    @property
    def patrol_success(self):
        self.update_state()
        return ((self.waypoints >= 2)
                & (self._env.episode_length_buf * self._env.step_dt >= self.cfg.success_duration_s)
                & ~self._env.termination_manager.terminated)

    def _write_command(self, ids):
        data = self.robot.data
        x = data.root_pos_w[ids, 0] - self._env.scene.env_origins[ids, 0]
        delta = self.target_x[ids] - x
        # Correct overshoot as well as deliberate reversals.
        heading = torch.where(delta >= 0, 0.0, math.pi)
        error = torch.atan2(torch.sin(heading - data.heading_w[ids]), torch.cos(heading - data.heading_w[ids]))
        near = delta.abs() < self.cfg.waypoint_tolerance
        stopping = self.sampled_stop[ids] | near
        self.vel_command_b[ids, 0] = torch.where(
            stopping, 0.0, self.sampled_speed[ids] * torch.cos(error).clamp(0.0, 1.0))
        self.vel_command_b[ids, 1] = 0.0
        self.vel_command_b[ids, 2] = torch.where(stopping, 0.0, (1.5 * error).clamp(-.8, .8))

    def _update_command(self):
        self.update_state()
        self._write_command(slice(None))

    def _update_metrics(self):
        super()._update_metrics()
        self.update_state()
        data = self.robot.data
        active = self._env.episode_length_buf > 0
        stopping = torch.linalg.vector_norm(self.command, dim=1) < .05
        speed = torch.linalg.vector_norm(data.root_lin_vel_w[:, :2], dim=1)
        xy = data.root_pos_w[:, :2]
        enter_stop = stopping & ~self.was_stopping
        self.stop_anchor[enter_stop] = xy[enter_stop]
        self.was_stopping[:] = stopping
        self.stop_samples += active & stopping
        for name, value in {
            'tilt_deg': torch.rad2deg(torch.acos((-data.projected_gravity_b[:, 2]).clamp(-1, 1))),
            'roll_pitch_speed': torch.linalg.vector_norm(data.root_ang_vel_b[:, :2], dim=1),
            'stop_fraction': stopping.float(),
            'climb_fraction': stair_climb_weight(self._env),
            'moving_stall_fraction': commanded_stall(self._env),
        }.items():
            self.metrics[name] += active * (value - self.metrics[name]) / self._motion_samples.clamp(min=1)
        for name, value in {
            'stop_speed': speed,
            'stop_position_drift': torch.linalg.vector_norm(xy - self.stop_anchor, dim=1),
            'stop_settled_fraction': ((speed < .15) & (data.projected_gravity_b[:, 2] < -.94)).float(),
        }.items():
            self.metrics[name] += (active & stopping) * (value - self.metrics[name]) / self.stop_samples.clamp(min=1)
        self.metrics['waypoints'][:] = self.waypoints
        self.metrics['long_episode_success'][:] = self.patrol_success.float()
        self.metrics['terrain_success'][:] = self.terrain_success.float()


@configclass
class StableTraversalVelocityCommandCfg(TraversalVelocityCommandCfg):
    class_type: type = StableTraversalVelocityCommand
    waypoint_x: float = 2.6
    waypoint_tolerance: float = .3
    settle_time_s: float = .6
    success_duration_s: float = 20.0
    slow_speed_range: tuple[float, float] = (.45, .65)
    robustness_delay_steps: int = 40000
    robustness_ramp_steps: int = 80000


def stable_waypoint_reward(env, command_name='base_velocity'):
    term = env.command_manager.get_term(command_name)
    term.update_state()
    return term.waypoint_pulse.float()


def stable_traversal_progress(env, command_name='base_velocity'):
    """Reward velocity along the commanded heading, including the return journey."""
    data = env.scene['robot'].data
    speed = env.command_manager.get_command(command_name)[:, 0]
    return torch.minimum(data.root_lin_vel_b[:, 0].clamp(min=-1), speed) * (-data.projected_gravity_b[:, 2]).clamp(0, 1)


def stair_climb_weight(env, command_name='base_velocity'):
    """Local upward edge near the wheels, in the robot's yaw frame.

    Distant roughness, down steps, missing rays and stop/turn commands do not
    relax stability. Wheel-bottom height avoids confusing crouching with a step.
    """
    robot = env.scene['robot']
    wheel_ids, _ = robot.find_bodies('Hub.*')
    wheels = robot.data.body_pos_w[:, wheel_ids]
    hits = env.scene.sensors['height_scanner'].data.ray_hits_w
    delta = hits[..., :2] - wheels[..., :2].mean(dim=1, keepdim=True)
    yaw = robot.data.heading_w[:, None]
    forward = delta[..., 0] * torch.cos(yaw) + delta[..., 1] * torch.sin(yaw)
    lateral = -delta[..., 0] * torch.sin(yaw) + delta[..., 1] * torch.cos(yaw)
    rise = hits[..., 2] - (wheels[..., 2].amin(dim=1, keepdim=True) - .0935)
    local = (torch.isfinite(hits).all(dim=-1) & (forward > -.08) & (forward < .65)
             & (lateral.abs() < .45) & (rise > .025) & (rise < .35))
    height = torch.where(local, rise, 0.0).amax(dim=1)
    command = env.command_manager.get_command(command_name)
    moving = (command[:, 0] > .15) & (command[:, 2].abs() < .7)
    upright = robot.data.projected_gravity_b[:, 2] < -math.cos(math.radians(45))
    return ((height - .02) / .04).clamp(0, 1) * moving * upright


def terrain_stability_orientation(env):
    """Permit deliberate climbing pitch, retain roll and recovery constraints."""
    gravity = env.scene['robot'].data.projected_gravity_b
    climb = stair_climb_weight(env)
    roll = (gravity[:, 1].abs() - math.sin(math.radians(8))).clamp(min=0)
    pitch_limit = torch.sin(torch.deg2rad(12 + 12 * climb))
    pitch = (gravity[:, 0].abs() - pitch_limit).clamp(min=0)
    return (roll.square() + (1 - .7 * climb) * pitch.square()) * (.35 + .65 * traversal_flat_weight(env))


def terrain_stability_ang_vel(env):
    speed = env.scene['robot'].data.root_ang_vel_b
    climb = stair_climb_weight(env)
    roll = (speed[:, 0].abs() - .3).clamp(min=0)
    pitch = (speed[:, 1].abs() - .3 - 1.2 * climb).clamp(min=0)
    return roll.square() + (1 - .7 * climb) * pitch.square()


def _high_speed_gate(env, command_name: str, start_speed: float, full_speed: float):
    """Return a smooth gate for stability penalties at commanded speed."""
    command = env.command_manager.get_command(command_name)
    speed = torch.linalg.vector_norm(command[:, :2], dim=1)
    return ((speed - start_speed) / max(full_speed - start_speed, 1e-6)).clamp(0.0, 1.0)


def high_speed_orientation_l2(
    env, command_name: str = "base_velocity", start_speed: float = 0.35, full_speed: float = 0.50
):
    """Penalize roll/pitch more strongly once the command enters the fast band."""
    gravity = env.scene["robot"].data.projected_gravity_b[:, :2]
    return _high_speed_gate(env, command_name, start_speed, full_speed) * gravity.square().sum(dim=1)


def high_speed_ang_vel_l2(
    env, command_name: str = "base_velocity", start_speed: float = 0.35, full_speed: float = 0.50
):
    """Damp fast roll/pitch oscillations without suppressing commanded yaw."""
    ang_vel = env.scene["robot"].data.root_ang_vel_b[:, :2]
    return _high_speed_gate(env, command_name, start_speed, full_speed) * ang_vel.square().sum(dim=1)


def high_speed_vertical_motion_l2(
    env, command_name: str = "base_velocity", start_speed: float = 0.35, full_speed: float = 0.50
):
    """Discourage bouncing while translating quickly over uneven terrain."""
    vertical = env.scene["robot"].data.root_lin_vel_b[:, 2]
    return _high_speed_gate(env, command_name, start_speed, full_speed) * vertical.square()


def terrain_action_rate(env):
    delta = env.action_manager.action - env.action_manager.prev_action
    # Four leg targets may change quickly at an edge. Wheel action smoothing
    # stays active to avoid replacing a leg lift with a wheel-speed impulse.
    climb = stair_climb_weight(env)
    return (1 - .7 * climb) * delta[:, :4].square().sum(dim=1) + delta[:, 4:].square().sum(dim=1)


def terrain_leg_deviation_l1(env, asset_cfg):
    """Keep the nominal stance, with local relief for deliberate climbing."""
    data = env.scene[asset_cfg.name].data
    deviation = (data.joint_pos[:, asset_cfg.joint_ids]
                 - data.default_joint_pos[:, asset_cfg.joint_ids]).abs().sum(dim=1)
    return (1 - .8 * stair_climb_weight(env)) * deviation


def commanded_stall(env, command_name='base_velocity'):
    command = env.command_manager.get_command(command_name)
    data = env.scene['robot'].data
    moving = (command[:, 0] > .2) & (command[:, 2].abs() < .3)
    return moving * ((.15 - data.root_lin_vel_b[:, 0]) / .15).clamp(0, 1)


class SupportedAscent(ManagerTermBase):
    """Pay each wheel's new supported elevation once per episode, in metres/s.

    Flight, crouching, repeated bouncing at an old height and parking cannot
    accumulate this bonus. Each wheel has its own support-height high-water mark.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.robot = env.scene['robot']
        self.wheel_ids, _ = self.robot.find_bodies('Hub.*')
        self.sensor = env.scene.sensors['contact_forces']
        self.sensor_ids, _ = self.sensor.find_bodies('Hub.*')
        shape = (env.num_envs, len(self.wheel_ids))
        self.high = torch.zeros(shape, device=env.device)
        self.initialized = torch.zeros(shape, device=env.device, dtype=torch.bool)

    def reset(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        self.initialized[ids] = False
        self.high[ids] = 0

    def __call__(self, env, command_name='base_velocity'):
        height = self.robot.data.body_pos_w[:, self.wheel_ids, 2] - .0935
        support = self.sensor.data.net_forces_w[:, self.sensor_ids, 2] > 5.0
        support &= self.sensor.data.current_air_time[:, self.sensor_ids] < .016
        new = support & ~self.initialized
        self.high[new] = height[new]
        self.initialized |= support
        gain = torch.where(support & (height > self.high + .01), height - self.high, 0.0)
        # Consume height changes even when stopped, so they cannot be banked
        # and paid out later by changing the command.
        self.high = torch.where(gain > 0, height, self.high)
        data = self.robot.data
        moving = ((env.command_manager.get_command(command_name)[:, 0] > .2)
                  & (data.root_lin_vel_b[:, 0] > .05)
                  & (data.projected_gravity_b[:, 2] < -.866))
        return gain.clamp(max=.22).mean(dim=1) * moving / env.step_dt


def grounded_vertical_motion(env):
    """Damp repeated grounded bouncing; leave ballistic jump flight unconstrained."""
    sensor = env.scene.sensors['contact_forces']
    hub_ids, _ = sensor.find_bodies('Hub.*')
    grounded = (sensor.data.current_air_time[:, hub_ids] < .016).all(dim=1)
    speed = (env.scene['robot'].data.root_lin_vel_w[:, 2].abs() - .2).clamp(min=0)
    return grounded * (1 - .9 * stair_climb_weight(env)) * (.2 + .8 * traversal_flat_weight(env)) * speed.square()


def stop_motion(env, command_name='base_velocity'):
    command = env.command_manager.get_command(command_name)
    data = env.scene['robot'].data
    stop = torch.linalg.vector_norm(command, dim=1) < .05
    return stop * (data.root_lin_vel_b.square().sum(dim=1) + .25 * data.root_ang_vel_b.square().sum(dim=1))


def excess_forward_speed(env, command_name='base_velocity'):
    command = env.command_manager.get_command(command_name)
    speed = env.scene['robot'].data.root_lin_vel_b[:, 0]
    margin = .2 + .2 * stair_climb_weight(env, command_name)
    return (speed - command[:, 0] - margin).clamp(min=0).square()


class RandomBodyForcePulse(ManagerTermBase):
    """Per-environment world-horizontal force pulses, explicitly cleared on expiry/reset.

    Forces scale with total robot mass: F = mass * sampled acceleration. They
    are applied to base_link, never by replacing velocities. Call every control
    step so forces cannot persist for an entire multi-second event interval.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.robot = env.scene['robot']
        self.body_ids, _ = self.robot.find_bodies('base_link')
        self.mass = self.robot.data.default_mass.sum(dim=1).to(env.device)
        self.forces = torch.zeros(env.num_envs, 1, 3, device=env.device)
        self.remaining = torch.zeros(env.num_envs, device=env.device)
        self.wait = torch.zeros_like(self.remaining)
        self.reset()

    def reset(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        self.forces[ids] = 0
        self.remaining[ids] = 0
        lo, hi = self.cfg.params.get('interval_s', (4.0, 7.0))
        self.wait[ids] = torch.empty_like(self.wait[ids]).uniform_(lo, hi)
        self._write()

    def _write(self):
        # Write ALL environments: a zero-only partial write would disable the
        # articulation's global has_external_wrench flag for still-active peers.
        self.robot.set_external_force_and_torque(
            self.forces, torch.zeros_like(self.forces), body_ids=self.body_ids, is_global=True)

    def __call__(self, env, env_ids, interval_s=(4.0, 7.0), duration_s=(.08, .20),
                 acceleration_range=(.5, 3.0), ramp_steps=100000, grace_s=2.0, delay_steps=0):
        # Configured as a global interval equal to step_dt; own clocks are per env.
        self.remaining.sub_(env.step_dt).clamp_(min=0)
        self.wait -= env.step_dt
        self.forces[self.remaining <= 0] = 0
        strength = robustness_strength(env, delay_steps, ramp_steps)
        start = ((self.wait <= 0) & (self.remaining <= 0) & (strength > 0)
                 & (env.episode_length_buf * env.step_dt >= grace_s))
        ids = start.nonzero().flatten()
        n = len(ids)
        duration = torch.empty(n, device=env.device).uniform_(*duration_s)
        acceleration = torch.empty(n, device=env.device).uniform_(*acceleration_range) * strength
        angle = torch.rand(n, device=env.device) * (2 * math.pi)
        self.forces[ids, 0, 0] = self.mass[ids] * acceleration * torch.cos(angle)
        self.forces[ids, 0, 1] = self.mass[ids] * acceleration * torch.sin(angle)
        self.remaining[ids] = duration
        self.wait[ids] = duration + torch.empty(n, device=env.device).uniform_(*interval_s)
        self._write()
