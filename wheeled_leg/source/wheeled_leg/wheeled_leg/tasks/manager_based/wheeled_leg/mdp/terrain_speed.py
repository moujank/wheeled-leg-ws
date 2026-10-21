# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Forward terrain commands with independent speed and traversal curricula."""

from __future__ import annotations

from collections.abc import Sequence

import torch
from isaaclab.envs.mdp.commands import UniformVelocityCommand, UniformVelocityCommandCfg
from isaaclab.utils import configclass


def _resolve_env_ids(num_envs, device, env_ids):
    ids = torch.arange(num_envs, device=device, dtype=torch.long)
    if env_ids is None:
        return ids
    if isinstance(env_ids, slice):
        return ids[env_ids]
    return torch.as_tensor(env_ids, device=device, dtype=torch.long).flatten()


class CurriculumVelocityCommand(UniformVelocityCommand):
    """One forward target per episode; face world +X, accelerate gradually.

    A forward scanner cannot support unobserved reverse traversal. Random yaw
    and reversals also let a policy spend an episode on the spawn platform.
    Hard rows retain the slow band while speed is learned on easy rows only.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        if not 0 <= cfg.initial_stage <= cfg.final_stage < len(cfg.stage_max_speeds):
            raise ValueError("Invalid initial/final speed stage")
        if min(cfg.stage_max_speeds) < cfg.min_forward_speed or cfg.max_acceleration <= 0:
            raise ValueError("Speed caps must exceed min_forward_speed; acceleration must be positive")
        if cfg.hard_terrain_max_speed < cfg.min_forward_speed or cfg.speed_training_stride < 1:
            raise ValueError("Hard-terrain cap must exceed min_forward_speed; cohort stride must be positive")
        self.stage_max_speeds = torch.as_tensor(cfg.stage_max_speeds, device=self.device)
        self.speed_stage = torch.full((self.num_envs,), cfg.initial_stage, device=self.device, dtype=torch.long)
        self.success_streak = torch.zeros_like(self.speed_stage)
        self.failure_streak = torch.zeros_like(self.speed_stage)
        self.sampled_speed = torch.zeros(self.num_envs, device=self.device)
        self.metric_samples = torch.zeros_like(self.sampled_speed)
        self.motion_samples = torch.zeros_like(self.sampled_speed)
        for name in ("tracking_error_mean", "command_speed_mean", "moving_tracking_error_mean",
                     "moving_time_s", "command_distance_m", "stall_fraction"):
            self.metrics[name] = torch.zeros_like(self.sampled_speed)

    def reset(self, env_ids=None):
        ids = _resolve_env_ids(self.num_envs, self.device, env_ids)
        self.metric_samples[ids] = 0
        self.motion_samples[ids] = 0
        self.vel_command_b[ids] = 0
        return super().reset(env_ids)

    def effective_max_speed(self):
        cap = self.stage_max_speeds[self.speed_stage]
        hard = self._env.scene.terrain.terrain_levels >= self.cfg.slow_terrain_level
        return torch.where(hard, cap.clamp(max=self.cfg.hard_terrain_max_speed), cap)

    def _resample_command(self, env_ids: Sequence[int]):
        ids = _resolve_env_ids(self.num_envs, self.device, env_ids)
        cap = self.effective_max_speed()[ids]
        self.sampled_speed[ids] = self.cfg.min_forward_speed + torch.rand(len(ids), device=self.device) * (
            cap - self.cfg.min_forward_speed
        )
        self.is_standing_env[ids] = torch.rand(len(ids), device=self.device) < self.cfg.rel_standing_envs
        # Explicitly keep targets through a full episode, independent of stage.
        self.time_left[ids] = self.cfg.resampling_time_range[0] + torch.rand(len(ids), device=self.device) * (
            self.cfg.resampling_time_range[1] - self.cfg.resampling_time_range[0]
        )

    def _update_command(self):
        target = torch.where(self.is_standing_env, 0.0, self.sampled_speed)
        delta = self.cfg.max_acceleration * self._env.step_dt
        self.vel_command_b[:, 0] += (target - self.vel_command_b[:, 0]).clamp(-delta, delta)
        self.vel_command_b[:, 1] = 0
        heading = self.robot.data.heading_w
        error = torch.atan2(-torch.sin(heading), torch.cos(heading))
        self.vel_command_b[:, 2] = (self.cfg.heading_control_stiffness * error).clamp(
            -self.cfg.max_heading_rate, self.cfg.max_heading_rate
        )
        self.vel_command_b[self.is_standing_env, 2] = 0

    def _update_metrics(self):
        super()._update_metrics()
        active = self._env.episode_length_buf > 0
        error = torch.linalg.vector_norm(self.vel_command_b[:, :2] - self.robot.data.root_lin_vel_b[:, :2], dim=1)
        speed = self.vel_command_b[:, 0]
        self.metric_samples += active
        denom = self.metric_samples.clamp(min=1)
        for name, value in (("tracking_error_mean", error), ("command_speed_mean", speed)):
            self.metrics[name] += active * (value - self.metrics[name]) / denom
        moving = active & (speed > .15)
        self.motion_samples += moving
        count = self.motion_samples.clamp(min=1)
        self.metrics["moving_tracking_error_mean"] += moving * (
            error - self.metrics["moving_tracking_error_mean"]
        ) / count
        stalled = (self.robot.data.root_lin_vel_b[:, 0] < .05).float()
        self.metrics["stall_fraction"] += moving * (stalled - self.metrics["stall_fraction"]) / count
        self.metrics["moving_time_s"] += moving * self._env.step_dt
        self.metrics["command_distance_m"] += active * speed * self._env.step_dt


@configclass
class CurriculumVelocityCommandCfg(UniformVelocityCommandCfg):
    class_type: type = CurriculumVelocityCommand
    # scale=4 is a multiplier, not a limit: clip +/-6 rad/s at r=.0935
    # permits ~.56 m/s ideally. Leave torque/acceleration/terrain headroom.
    stage_max_speeds: tuple[float, ...] = (.25, .30, .35, .40)
    initial_stage: int = 0
    final_stage: int = 3
    min_forward_speed: float = .22
    max_acceleration: float = .30
    max_heading_rate: float = .30
    slow_terrain_level: int = 3
    hard_terrain_max_speed: float = .25
    # Every fifth environment remains on easy rows to retain speed coverage
    # after the other environments have advanced to difficult terrain.
    speed_training_stride: int = 5


def terrain_speed_curriculum(
    env, env_ids, command_name="base_velocity", min_episode_duration=8.0,
    tracking_error_threshold=.07, min_command_speed=.18,
    successes_to_promote=4, failures_to_demote=2,
):
    """Evaluate moving exposure, leaving standing/hard-row episodes neutral.

    This term must run BEFORE terrain_levels so it sees the completed row.
    Falls always count as failures, including falls on hard terrain.
    """
    term = env.command_manager.get_term(command_name)
    ids = _resolve_env_ids(env.num_envs, env.device, env_ids)
    ids = ids[env.episode_length_buf[ids] > 0]
    if ids.numel():
        metrics = term.metrics
        failed = env.termination_manager.terminated[ids]
        exposed = ((metrics["moving_time_s"][ids] >= min_episode_duration)
                   & (metrics["command_speed_mean"][ids] >= min_command_speed)
                   & (env.scene.terrain.terrain_levels[ids] < term.cfg.slow_terrain_level))
        evaluated = exposed | failed
        good = (exposed & env.termination_manager.time_outs[ids] & ~failed
                & (metrics["moving_tracking_error_mean"][ids] <= tracking_error_threshold)
                & (metrics["stall_fraction"][ids] < .10))
        success = torch.where(good, term.success_streak[ids] + 1, 0)
        failure = torch.where(good, 0, term.failure_streak[ids] + 1)
        term.success_streak[ids] = torch.where(evaluated, success, term.success_streak[ids])
        term.failure_streak[ids] = torch.where(evaluated, failure, term.failure_streak[ids])
        up = (term.success_streak[ids] >= successes_to_promote) & (term.speed_stage[ids] < term.cfg.final_stage)
        down = (term.failure_streak[ids] >= failures_to_demote) & (term.speed_stage[ids] > 0)
        term.speed_stage[ids] += up.long() - down.long()
        changed = ids[up | down]
        term.success_streak[changed] = 0
        term.failure_streak[changed] = 0
    return {
        "mean_stage": term.speed_stage.float().mean(),
        "mean_max_speed_mps": term.effective_max_speed().mean(),
        "final_stage_fraction": (term.speed_stage >= term.cfg.final_stage).float().mean(),
    }


__all__ = ["CurriculumVelocityCommand", "CurriculumVelocityCommandCfg", "terrain_speed_curriculum"]
