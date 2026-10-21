# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Success-driven stationary -> moving -> obstacle jump course.

The command owns per-episode evidence. All reward/termination/observation calls
share one update per episode step, so reward ordering cannot double count flight.
"""

import math
import torch
from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.utils import configclass


def _yaw(robot):
    w, x, y, z = robot.data.root_quat_w.unbind(-1)
    return torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class JumpCourseCommand(CommandTerm):
    """A three-value velocity command plus explicit goal and jump reference state."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.robot = env.scene['robot']
        self.sensor = env.scene.sensors['contact_forces']
        self.wheel_ids, _ = self.robot.find_bodies('Hub.*')
        self.wheel_contact_ids, _ = self.sensor.find_bodies('Hub.*')
        self.other_contact_ids, _ = self.sensor.find_bodies('base_link|Hip.*|Knee.*')
        if len(self.wheel_ids) != 2 or len(self.wheel_contact_ids) != 2:
            raise ValueError('The jump course requires exactly two wheel bodies/contact sensors.')
        n, dev = self.num_envs, self.device
        self.level = torch.full((n,), cfg.initial_stage, dtype=torch.long, device=dev)
        self.success_ema = torch.zeros(n, device=dev)
        self.attempts = torch.zeros(n, dtype=torch.long, device=dev)
        self.goal_w = torch.zeros(n, 3, device=dev)
        self.barrier_x = torch.zeros(n, device=dev)
        self.barrier_height = torch.zeros(n, device=dev)
        self.target_clearance = torch.zeros(n, device=dev)
        self.trigger_time = torch.full((n,), -1.0, device=dev)
        self._command = torch.zeros(n, 3, device=dev)
        self.last_step = torch.full((n,), -1, dtype=torch.long, device=dev)
        self.ground_time = torch.zeros(n, device=dev)
        self.landing_time = torch.zeros(n, device=dev)
        self.armed = torch.zeros(n, dtype=torch.bool, device=dev)
        self.in_flight = torch.zeros(n, dtype=torch.bool, device=dev)
        self.flight_time = torch.zeros(n, device=dev)
        self.flight_ended = torch.zeros(n, dtype=torch.bool, device=dev)
        self.peak_clearance = torch.zeros(n, device=dev)
        self.takeoff_x = torch.zeros(n, device=dev)
        self.invalid = torch.zeros(n, dtype=torch.bool, device=dev)
        self.entered = torch.zeros(n, 2, dtype=torch.bool, device=dev)
        self.cleared = torch.zeros(n, 2, dtype=torch.bool, device=dev)
        self.previous_wheel_x = torch.zeros(n, 2, device=dev)
        self.crossed = torch.zeros(n, dtype=torch.bool, device=dev)
        self.jump_landed = torch.zeros(n, dtype=torch.bool, device=dev)
        self.successful = torch.zeros(n, dtype=torch.bool, device=dev)
        self.success_pulse = torch.zeros(n, dtype=torch.bool, device=dev)
        self.landing_pulse = torch.zeros(n, dtype=torch.bool, device=dev)
        self.metrics = {key: torch.zeros(n, device=dev) for key in (
            'stage', 'success_rate', 'barrier_height', 'goal_distance', 'jump_landed', 'strict_success',
        )}

    @property
    def command(self):
        return self._command

    @property
    def moving(self):
        return self.level >= 3

    def reference(self):
        time = self._env.episode_length_buf.float() * self._env.step_dt
        duration = (2 * torch.sqrt(2 * (self.target_clearance + 0.04) / 9.81)).clamp(min=0.28)
        stationary_clock = torch.remainder(time, self.cfg.stationary_period_s) - self.cfg.takeoff_s
        moving_clock = time - self.trigger_time - self.cfg.crouch_s
        clock = torch.where(self.moving, moving_clock, stationary_clock)
        enabled = (~self.moving) | (self.trigger_time >= 0)
        preparation = enabled & (clock >= -self.cfg.crouch_s) & (clock < 0)
        flight = enabled & (clock >= 0) & (clock < duration)
        crouch_fraction = ((clock + self.cfg.crouch_s) / self.cfg.crouch_s).clamp(0, 1)
        phase = (clock / duration).clamp(0, 1)
        target = torch.full_like(time, self.cfg.nominal_height)
        target = torch.where(preparation, target - self.cfg.crouch_depth * crouch_fraction, target)
        target = torch.where(flight, target + 4 * self.target_clearance * phase * (1 - phase), target)
        return clock, preparation, flight, target

    def _resample_command(self, env_ids):
        ids = (torch.arange(self.num_envs, device=self.device)[env_ids] if isinstance(env_ids, slice)
               else torch.as_tensor(env_ids, dtype=torch.long, device=self.device))
        level = self.level[ids]
        heights = torch.tensor(self.cfg.stationary_heights, device=self.device)
        barriers = torch.tensor(self.cfg.barrier_heights, device=self.device)
        self.barrier_height[ids] = torch.where(level >= 4, barriers[(level - 4).clamp(0, 4)], 0.0)
        self.target_clearance[ids] = torch.where(
            level < 3, heights[level.clamp(max=2)],
            torch.where(level == 3, torch.full_like(level, 0.26, dtype=torch.float), (self.barrier_height[ids] + 0.14).clamp(min=0.26)),
        )
        origins = self._env.scene.env_origins[ids]
        self.goal_w[ids] = origins
        self.goal_w[ids, 0] += torch.where(level >= 3, self.cfg.goal_x, 0.0)
        self.goal_w[ids, 2] += 0.03
        self.barrier_x[ids] = origins[:, 0] + self.cfg.barrier_x
        obstacle = self._env.scene['jump_barrier']
        pose = obstacle.data.default_root_state[ids, :7].clone()
        pose[:, :3] = origins
        pose[:, 0] = self.barrier_x[ids]
        # Fixed 20 cm cuboid: bury its lower portion to change the exposed height.
        pose[:, 2] += torch.where(level >= 4, self.barrier_height[ids] - 0.10, -0.50)
        obstacle.write_root_pose_to_sim(pose, env_ids=ids)
        obstacle.write_root_velocity_to_sim(torch.zeros((len(ids), 6), device=self.device), env_ids=ids)
        self.trigger_time[ids] = -1.0
        self.last_step[ids] = -1
        for name in ('ground_time', 'landing_time', 'flight_time', 'peak_clearance', 'takeoff_x'):
            getattr(self, name)[ids] = 0
        for name in ('armed', 'in_flight', 'flight_ended', 'invalid', 'entered', 'cleared', 'crossed',
                     'jump_landed', 'successful', 'success_pulse', 'landing_pulse'):
            getattr(self, name)[ids] = False
        self.previous_wheel_x[ids] = self.robot.data.body_pos_w[ids][:, self.wheel_ids, 0]
        self._command[ids] = 0
        # A reset observation must contain the new goal velocity immediately.
        self._update_velocity_command()

    def update_state(self):
        step = self._env.episode_length_buf
        fresh = step != self.last_step
        if not fresh.any():
            return
        self.last_step.copy_(step)
        self.success_pulse[fresh] = False
        self.landing_pulse[fresh] = False
        dt = self._env.step_dt
        time = step.float() * dt
        pos = self.robot.data.root_pos_w
        trigger = fresh & self.moving & (self.trigger_time < 0) & (pos[:, 0] >= self.barrier_x - self.cfg.launch_distance)
        self.trigger_time[trigger] = time[trigger]
        clock, preparation, flight, _ = self.reference()
        air = (self.sensor.data.current_air_time[:, self.wheel_contact_ids] > 0.015).all(-1)
        grounded = (self.sensor.data.current_contact_time[:, self.wheel_contact_ids] > 0.02).all(-1)
        nonwheel_contact = (self.sensor.data.net_forces_w[:, self.other_contact_ids].norm(dim=-1) > 1.0).any(-1)
        self.ground_time[fresh] = torch.where(grounded[fresh], self.ground_time[fresh] + dt, 0.0)
        self.armed |= fresh & (self.ground_time >= 0.10)
        wheels = self.robot.data.body_pos_w[:, self.wheel_ids]
        bottom = wheels[..., 2] - self.cfg.wheel_radius - self._env.scene.env_origins[:, 2:3]
        front = self.barrier_x - self.cfg.barrier_width / 2 - self.cfg.wheel_radius
        back = self.barrier_x + self.cfg.barrier_width / 2 + self.cfg.wheel_radius
        start = fresh & air & self.armed & (~self.in_flight) & (~self.jump_landed) & (time > 0.5) & (preparation | flight)
        start &= (~self.moving) | (wheels[..., 0].amax(-1) < front)
        self.in_flight |= start
        self.armed[start] = False
        self.takeoff_x[start] = pos[start, 0]
        active = fresh & self.in_flight
        # A jump must be a single uninterrupted dual-wheel flight. Landing one
        # wheel and hopping the other over the bar cannot complete the evidence.
        self.flight_ended |= active & (~air)
        flying = active & air & (~self.flight_ended)
        self.flight_time[flying] += dt
        self.peak_clearance[flying] = torch.maximum(self.peak_clearance[flying], bottom[flying].amin(-1))
        # Invalid evidence stays latched through landing; driving/climbing on the
        # cuboid, jumping alongside it, or touching with a leg cannot count.
        overlap = (wheels[..., 0] >= front[:, None]) & (wheels[..., 0] <= back[:, None])
        high = bottom > self.barrier_height[:, None] + self.cfg.clearance_margin
        lane = (wheels[..., 1] - self._env.scene.env_origins[:, 1:2]).abs() < self.cfg.lane_half_width
        collision = (overlap & (~high | ~lane)).any(-1) | (overlap.any(-1) & ~air)
        self.invalid |= active & (nonwheel_contact | (self.moving & collision))
        entered = (self.previous_wheel_x < front[:, None]) & (wheels[..., 0] >= front[:, None])
        self.entered |= flying[:, None] & high & lane & entered
        self.cleared |= flying[:, None] & self.entered & high & lane & (wheels[..., 0] > back[:, None])
        self.crossed |= flying & self.cleared.all(-1) & (~self.invalid)
        self.invalid |= active & self.moving & self.flight_ended & (~self.crossed)
        self.crossed &= ~self.invalid
        self.landing_time[fresh] = torch.where(grounded[fresh] & self.in_flight[fresh], self.landing_time[fresh] + dt, 0.0)
        landed = active & (self.landing_time >= self.cfg.landing_stable_s)
        upright = self.robot.data.projected_gravity_b[:, 2] < -0.85
        stationary_ok = (pos[:, :2] - self._env.scene.env_origins[:, :2]).norm(dim=-1) < 0.25
        valid = landed & (self.flight_time >= 0.08) & (~self.invalid) & upright
        valid &= self.peak_clearance >= self.target_clearance - 0.015
        valid &= torch.where(self.moving, self.crossed & (wheels[..., 0].amin(-1) > back), stationary_ok)
        self.jump_landed |= valid
        self.landing_pulse |= valid
        # A failed attempt ends flight evidence; permit another stationary cycle.
        self.in_flight[landed] = False
        failed = landed & ~valid
        self.flight_time[failed] = 0
        self.peak_clearance[failed] = 0
        self.invalid[failed] = False
        self.flight_ended[failed] = False
        self.entered[failed] = False
        self.cleared[failed] = False
        self.crossed[failed] = False
        self.landing_time[failed] = 0
        at_goal = (pos[:, :2] - self.goal_w[:, :2]).norm(dim=-1) < self.cfg.goal_tolerance
        speed = self.robot.data.root_lin_vel_w[:, :2].norm(dim=-1) < 0.35
        success = fresh & self.jump_landed & grounded & upright & at_goal & speed
        self.success_pulse |= success & (~self.successful)
        self.successful |= success
        self.previous_wheel_x[fresh] = wheels[fresh, :, 0]
        # Terminations run before command.compute(). Publish terminal evidence now
        # so CommandTerm.reset() logs success before it resamples these buffers.
        self._refresh_metrics()

    def _update_command(self):
        self.update_state()
        self._update_velocity_command()

    def _update_velocity_command(self):
        delta = self.goal_w[:, :2] - self.robot.data.root_pos_w[:, :2]
        yaw = _yaw(self.robot)
        distance = delta.norm(dim=-1)
        heading = torch.atan2(delta[:, 1], delta[:, 0]) - yaw
        heading = torch.atan2(heading.sin(), heading.cos())
        speed = torch.full_like(distance, self.cfg.approach_speed)
        self._command[:, 0] = torch.minimum(speed, 1.5 * distance) * self.moving
        self._command[:, 0] *= heading.cos().clamp(0, 1)
        self._command[:, 1] = 0
        self._command[:, 2] = (2 * heading).clamp(-1, 1) * self.moving * (distance > self.cfg.goal_tolerance)

    def _update_metrics(self):
        self.update_state()
        self._refresh_metrics()

    def _refresh_metrics(self):
        self.metrics['stage'].copy_(self.level)
        self.metrics['success_rate'].copy_(self.success_ema)
        self.metrics['barrier_height'].copy_(self.barrier_height)
        self.metrics['goal_distance'].copy_((self.goal_w[:, :2] - self.robot.data.root_pos_w[:, :2]).norm(dim=-1))
        self.metrics['jump_landed'].copy_(self.jump_landed)
        self.metrics['strict_success'].copy_(self.successful)

    def _set_debug_vis_impl(self, debug_vis):
        if debug_vis and not hasattr(self, 'goal_markers'):
            from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
            import isaaclab.sim as sim_utils
            self.goal_markers = VisualizationMarkers(VisualizationMarkersCfg(
                prim_path='/Visuals/JumpCourse',
                markers={
                    'start': sim_utils.SphereCfg(radius=0.06, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.3, 1.0))),
                    'goal': sim_utils.SphereCfg(radius=0.08, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 1.0, 0.1))),
                },
            ))
        if hasattr(self, 'goal_markers'):
            self.goal_markers.set_visibility(debug_vis)

    def _debug_vis_callback(self, event):
        if not hasattr(self, 'goal_w'):
            return
        starts = self._env.scene.env_origins.clone()
        starts[:, 2] += 0.03
        self.goal_markers.visualize(
            torch.cat((starts, self.goal_w)),
            marker_indices=[0] * self.num_envs + [1] * self.num_envs,
        )


@configclass
class JumpCourseCommandCfg(CommandTermCfg):
    class_type: type = JumpCourseCommand
    initial_stage: int = 0
    stationary_heights: tuple[float, ...] = (0.04, 0.08, 0.12)
    barrier_heights: tuple[float, ...] = (0.04, 0.08, 0.12, 0.16, 0.20)
    stationary_period_s: float = 2.8
    takeoff_s: float = 1.2
    crouch_s: float = 0.35
    crouch_depth: float = 0.10
    nominal_height: float = 0.55
    wheel_radius: float = 0.0935
    barrier_x: float = 1.3
    barrier_width: float = 0.10
    # At 0.95 m/s, allow 0.35 s crouch plus flight time to the bar.
    launch_distance: float = 0.75
    approach_speed: float = 0.95
    goal_x: float = 2.6
    goal_tolerance: float = 0.25
    lane_half_width: float = 0.60
    landing_stable_s: float = 0.12
    clearance_margin: float = 0.015


def jump_course(env):
    term = env.command_manager.get_term('base_velocity')
    term.update_state()
    return term


def jump_course_observation(env):
    term = jump_course(env)
    robot = term.robot
    yaw = _yaw(robot)
    delta = term.goal_w[:, :2] - robot.data.root_pos_w[:, :2]
    goal_x = yaw.cos() * delta[:, 0] + yaw.sin() * delta[:, 1]
    goal_y = -yaw.sin() * delta[:, 0] + yaw.cos() * delta[:, 1]
    clock, prep, flight, target = term.reference()
    base_height = robot.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
    air = term.sensor.data.current_air_time[:, term.wheel_contact_ids].clamp(0, 1)
    return torch.stack((
        goal_x.clamp(-4, 4), goal_y.clamp(-4, 4),
        (term.barrier_x - robot.data.root_pos_w[:, 0]).clamp(-4, 4),
        term.barrier_height, torch.full_like(clock, term.cfg.barrier_width), term.level.float() / 8,
        (prep | flight).float(), clock.clamp(-1, 1), target - term.cfg.nominal_height,
        base_height - term.cfg.nominal_height, air[:, 0], air[:, 1],
    ), dim=-1)


def jump_height_tracking(env):
    term = jump_course(env)
    _, prep, flight, target = term.reference()
    height = term.robot.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
    upright = (-term.robot.data.projected_gravity_b[:, 2]).clamp(0, 1)
    return torch.exp(-((height - target) / 0.10).square()) * upright * (prep | flight)


def jump_flight_reward(env):
    term = jump_course(env)
    _, _, flight, _ = term.reference()
    air = (term.sensor.data.current_air_time[:, term.wheel_contact_ids] > 0.015).all(-1)
    bottom = term.robot.data.body_pos_w[:, term.wheel_ids, 2].amin(-1) - term.cfg.wheel_radius - env.scene.env_origins[:, 2]
    return (bottom / term.target_clearance.clamp(min=0.04)).clamp(0, 1) * air * flight * (~term.invalid)


def jump_launch_velocity(env):
    term = jump_course(env)
    clock, _, _, _ = term.reference()
    wanted = torch.sqrt(2 * 9.81 * term.target_clearance)
    velocity = term.robot.data.root_lin_vel_w[:, 2]
    return torch.exp(-((velocity - wanted) / 0.8).square()) * (clock >= -0.05) * (clock < 0.12)


def jump_goal_progress(env):
    term = jump_course(env)
    direction = term.goal_w[:, :2] - term.robot.data.root_pos_w[:, :2]
    direction /= direction.norm(dim=-1, keepdim=True).clamp(min=0.1)
    progress = (term.robot.data.root_lin_vel_w[:, :2] * direction).sum(-1).clamp(-1, 1.5)
    # After an unqualified crossing, no forward credit can be collected at the goal.
    permitted = (term.robot.data.root_pos_w[:, 0] < term.barrier_x) | term.crossed
    return progress * term.moving * permitted


def jump_landing_reward(env):
    return jump_course(env).landing_pulse.float()


def jump_success_reward(env):
    return jump_course(env).success_pulse.float()


def jump_success(env):
    return jump_course(env).successful


def jump_recovery_velocity_l2(env):
    term = jump_course(env)
    _, prep, flight, _ = term.reference()
    return term.robot.data.root_lin_vel_w[:, 2].square() * (~(prep | flight))


def jump_stationary_drift(env):
    term = jump_course(env)
    drift = term.robot.data.root_pos_w[:, :2] - env.scene.env_origins[:, :2]
    return drift.square().sum(-1) * (~term.moving)


def jump_course_out_of_bounds(env):
    term = jump_course(env)
    delta = term.robot.data.root_pos_w - env.scene.env_origins
    return (delta[:, 1].abs() > term.cfg.lane_half_width) | (delta[:, 0] < -0.7) | (delta[:, 0] > term.cfg.goal_x + 0.8)


def jump_failed_crossing(env):
    term = jump_course(env)
    past = term.robot.data.body_pos_w[:, term.wheel_ids, 0].amin(-1) > term.barrier_x + term.cfg.barrier_width / 2 + term.cfg.wheel_radius
    return term.moving & past & (~term.crossed)


def jump_course_curriculum(env, env_ids, min_episodes: int = 20, success_threshold: float = 0.7):
    term = env.command_manager.get_term('base_velocity')
    if isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
    if env_ids is not None and len(env_ids):
        ids = env_ids[env.episode_length_buf[env_ids] > 0]
        term.attempts[ids] += 1
        success = term.successful[ids] & ~env.termination_manager.terminated[ids]
        term.success_ema[ids] = 0.9 * term.success_ema[ids] + 0.1 * success.float()
        promote = (term.attempts[ids] >= min_episodes) & (term.success_ema[ids] >= success_threshold) & (term.level[ids] < 8)
        promoted = ids[promote]
        term.level[promoted] += 1
        term.attempts[promoted] = 0
        term.success_ema[promoted] = 0
    return {'mean_stage': term.level.float().mean(), 'mean_success_rate': term.success_ema.mean(),
            'stationary_fraction': (term.level < 3).float().mean(), 'full_height_fraction': (term.level == 8).float().mean()}
