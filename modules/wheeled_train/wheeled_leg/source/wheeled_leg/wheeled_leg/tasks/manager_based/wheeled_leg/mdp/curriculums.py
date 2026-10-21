# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Performance-based terrain curriculum for stable wheeled-leg motion."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def terrain_levels_stable_tracking(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    min_distance: float = 2.0,
    tracking_threshold: float = 0.75,
    yaw_tracking_threshold: float = 0.7,
    min_command_speed: float = 0.1,
    min_episode_duration: float = 5.0,
    distance_fraction: float = 0.6,
    min_traversal_distance: float = 0.75,
    command_name: str = "base_velocity",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Promote stable traversal, accounting for slower commands and curved motion.

    Tracking scores are normalized by the actual episode duration and reward weights.
    Commands must remain constant during each episode. The expected endpoint distance
    is the chord of the commanded circular trajectory, with a straight-line limit.
    A minimum actual displacement prevents promotion for standing or small circles.
    """
    terrain = env.scene.terrain
    if isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
    env_ids = torch.as_tensor(env_ids, device=env.device, dtype=torch.long)
    # The first reset has no completed episode to evaluate.
    env_ids = env_ids[env.episode_length_buf[env_ids] > 0]
    if env_ids.numel() == 0:
        return terrain.terrain_levels.float().mean()

    asset = env.scene[asset_cfg.name]
    distance = torch.linalg.vector_norm(
        asset.data.root_pos_w[env_ids, :2] - env.scene.env_origins[env_ids, :2], dim=1,
    )
    duration = env.episode_length_buf[env_ids].float() * env.step_dt
    command = env.command_manager.get_command(command_name)[env_ids]
    command_speed = torch.linalg.vector_norm(command[:, :2], dim=1)
    # torch.sinc(x) = sin(pi*x)/(pi*x), including the zero-yaw limit.
    expected_distance = command_speed * duration * torch.abs(
        torch.sinc(command[:, 2] * duration / (2.0 * torch.pi))
    )
    required_distance = torch.clamp(
        distance_fraction * expected_distance, min=min_traversal_distance, max=min_distance,
    )
    lin_weight = env.reward_manager.get_term_cfg("track_lin_vel_xy_exp").weight
    yaw_weight = env.reward_manager.get_term_cfg("track_ang_vel_z_exp").weight
    lin_score = env.reward_manager._episode_sums["track_lin_vel_xy_exp"][env_ids] / (duration * lin_weight)
    yaw_score = env.reward_manager._episode_sums["track_ang_vel_z_exp"][env_ids] / (duration * yaw_weight)
    failed = env.termination_manager.terminated[env_ids]
    completed = env.termination_manager.time_outs[env_ids] & ~failed
    move_up = (
        completed
        & (command_speed >= min_command_speed)
        & (duration >= min_episode_duration)
        & (distance > required_distance)
        & (lin_score > tracking_threshold)
        & (yaw_score > yaw_tracking_threshold)
    )
    terrain.update_env_origins(env_ids, move_up, failed)
    return terrain.terrain_levels.float().mean()


def terrain_level_distribution(env: ManagerBasedRLEnv, env_ids: Sequence[int]) -> dict[str, torch.Tensor]:
    """Log level occupancy and per-type means using the generator's column mapping.

    The global mean includes flat terrain, whose geometry never becomes harder.
    These statistics describe current assignments, not episode success rates.
    """
    terrain = env.scene.terrain
    cfg = terrain.cfg.terrain_generator
    levels = terrain.terrain_levels.float()
    stats = {
        "level_0_fraction": (levels == 0).float().mean(),
        "top_level_fraction": (levels == cfg.num_rows - 1).float().mean(),
    }
    # Match TerrainGenerator._generate_curriculum_terrains exactly.
    proportions = torch.tensor(
        [sub_cfg.proportion for sub_cfg in cfg.sub_terrains.values()], device=env.device,
    )
    boundaries = torch.cumsum(proportions / proportions.sum(), dim=0)
    column_samples = torch.arange(cfg.num_cols, device=env.device) / cfg.num_cols + 0.001
    column_types = torch.searchsorted(boundaries, column_samples, right=True)
    env_types = column_types[terrain.terrain_types]
    for index, name in enumerate(cfg.sub_terrains):
        mask = env_types == index
        if mask.any():
            stats[f"{name}_mean_level"] = levels[mask].mean()
    if "flat" in cfg.sub_terrains:
        nonflat = env_types != list(cfg.sub_terrains).index("flat")
        if nonflat.any():
            stats["nonflat_mean_level"] = levels[nonflat].mean()
    return stats


def terrain_levels_forward_traversal(
    env, env_ids, command_name="base_velocity", min_distance=2.6,
    min_episode_duration=8.0, distance_fraction=.65, tracking_error_threshold=.10,
    successes_to_promote=2, max_lateral_distance=1.25, update_levels=True,
    retain_speed_cohort=True,
):
    """Promote sustained traversal, using integrated commands rather than the last sample.

    Counters are cumulative, by terrain type, and are recorded on the completed
    row BEFORE changing origins. Difference counters over a window for rates.
    Top-row successes stay at the top; stops are neutral; falls demote one row.
    A small fixed cohort stays on easy rows for the independent speed curriculum.
    With update_levels=False, record the same outcomes on fixed evaluation rows.
    """
    terrain = env.scene.terrain
    cfg = terrain.cfg.terrain_generator
    term = env.command_manager.get_term(command_name)
    ids = torch.arange(env.num_envs, device=env.device)
    ids = ids if env_ids is None else ids[env_ids]
    ids = ids[env.episode_length_buf[ids] > 0]
    if not hasattr(env, "_forward_terrain_streak"):
        env._forward_terrain_streak = torch.zeros(env.num_envs, device=env.device, dtype=torch.long)
        # type x {all attempts, all successes, top attempts, top successes}
        env._forward_terrain_counts = torch.zeros(len(cfg.sub_terrains), 4, device=env.device)
        env._forward_last_success = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    if ids.numel():
        metrics = term.metrics
        local = env.scene["robot"].data.root_pos_w[ids] - env.scene.env_origins[ids]
        failed = env.termination_manager.terminated[ids]
        duration = env.episode_length_buf[ids].float() * env.step_dt
        # Full standing episodes are neither evidence of traversal nor failures.
        exposed = metrics["command_distance_m"][ids] >= min_distance
        attempted = exposed | failed
        success = (
            exposed & ~failed & env.termination_manager.time_outs[ids]
            & (duration >= min_episode_duration)
            & (local[:, 0] >= min_distance)
            & (local[:, 1].abs() < max_lateral_distance)
            & (local[:, 0] >= distance_fraction * metrics["command_distance_m"][ids])
            & (metrics["moving_tracking_error_mean"][ids] < tracking_error_threshold)
            & (env.scene["robot"].data.projected_gravity_b[ids, 2] < -.707)
        )
        env._forward_last_success[ids] = success
        proportions = torch.tensor([s.proportion for s in cfg.sub_terrains.values()], device=env.device)
        boundaries = torch.cumsum(proportions / proportions.sum(), dim=0)
        samples = torch.arange(cfg.num_cols, device=env.device) / cfg.num_cols + .001
        types = torch.searchsorted(boundaries, samples, right=True)[terrain.terrain_types[ids]]
        top = terrain.terrain_levels[ids] == cfg.num_rows - 1
        for index in range(len(cfg.sub_terrains)):
            mask = types == index
            env._forward_terrain_counts[index] += torch.stack((
                (mask & attempted).sum(), (mask & success).sum(),
                (mask & top & attempted).sum(), (mask & top & success).sum(),
            ))
        streak = torch.where(success, env._forward_terrain_streak[ids] + 1, 0)
        streak = torch.where(attempted, streak, env._forward_terrain_streak[ids])
        promote = streak >= successes_to_promote
        env._forward_terrain_streak[ids] = torch.where(promote, 0, streak)
        easy_cohort = (ids.remainder(term.cfg.speed_training_stride) == 0) & retain_speed_cohort
        ceiling = torch.where(easy_cohort, term.cfg.slow_terrain_level - 1, cfg.num_rows - 1)
        up = promote & (terrain.terrain_levels[ids] < ceiling)
        # A completed, sufficiently commanded stall also needs an easier retry.
        down = failed | (exposed & ~success)
        if update_levels:
            terrain.update_env_origins(ids, up, down)
            env.scene.env_origins[ids] = terrain.env_origins[ids]
    stats = {"mean_level": terrain.terrain_levels.float().mean()}
    for i, name in enumerate(cfg.sub_terrains):
        for j, key in enumerate(("attempts", "successes", "top_attempts", "top_successes")):
            stats[f"{name}_{key}"] = env._forward_terrain_counts[i, j].clone()
    return stats
