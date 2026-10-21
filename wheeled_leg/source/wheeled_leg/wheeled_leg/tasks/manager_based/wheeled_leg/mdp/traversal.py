# SPDX-License-Identifier: BSD-3-Clause
"""Progress and success for traversal without prescribing a contact/gait pattern."""

import torch
from isaaclab.managers import SceneEntityCfg


def traversal_progress(env, command_name: str = 'base_velocity'):
    """Reward world-X progress only up to the requested forward speed."""
    robot = env.scene['robot']
    upright = (-robot.data.projected_gravity_b[:, 2]).clamp(0.0, 1.0)
    speed = env.command_manager.get_command(command_name)[:, 0].clamp(min=0.0)
    return torch.minimum(robot.data.root_lin_vel_w[:, 0].clamp(min=-1.0), speed) * upright


def traversal_flat_weight(env, relief_threshold: float = 0.06):
    """Relax rolling preferences near steps; use raw hits so missing rays are never flat."""
    hits = env.scene.sensors['height_scanner'].data.ray_hits_w[..., 2]
    valid = torch.isfinite(hits).all(dim=1)
    heights = torch.nan_to_num(hits, nan=0.0, posinf=0.0, neginf=0.0)
    relief = heights.amax(dim=1) - heights.amin(dim=1)
    return (1.0 - relief / relief_threshold).clamp(0.0, 1.0) * valid


def traversal_flat_vertical_velocity(env, deadband: float = 0.15):
    """Mild cost of bouncing on flat ground, with free small terrain corrections."""
    excess = (env.scene['robot'].data.root_lin_vel_w[:, 2].abs() - deadband).clamp(min=0.0)
    return traversal_flat_weight(env) * excess.square()


def traversal_flat_leg_motion(env, asset_cfg: SceneEntityCfg):
    return traversal_flat_weight(env) * env.scene[asset_cfg.name].data.joint_vel[:, asset_cfg.joint_ids].square().sum(dim=1)


def traversal_lateral_deviation(env):
    # Soft guidance leaves room to choose a foothold on random squares.
    return (env.scene['robot'].data.root_pos_w[:, 1] - env.scene.env_origins[:, 1]).square()


def traversal_success(env, distance: float = 2.8):
    robot = env.scene['robot']
    local = robot.data.root_pos_w - env.scene.env_origins
    return (local[:, 0] > distance) & (local[:, 1].abs() < 3.5) & (robot.data.projected_gravity_b[:, 2] < -0.5)


def traversal_curriculum(env, env_ids, successes_to_promote: int = 2, success_command_name: str | None = None,
                        success_attribute: str = 'patrol_success'):
    """Two consecutive completions promote; every completed failure demotes.

    Keep mastered environments at the hardest row rather than invoking the
    importer's default random wrap. Initial resets never alter difficulty.
    """
    terrain = env.scene.terrain
    if not hasattr(env, '_traversal_success_streak'):
        env._traversal_success_streak = torch.zeros(env.num_envs, device=env.device, dtype=torch.long)
    if isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    env_ids = torch.as_tensor(env_ids, device=env.device, dtype=torch.long)
    env_ids = env_ids[env.episode_length_buf[env_ids] > 0]
    if env_ids.numel():
        completed = (traversal_success(env) if success_command_name is None
                     else getattr(env.command_manager.get_term(success_command_name), success_attribute))
        success = completed[env_ids] & ~env.termination_manager.terminated[env_ids]
        streak = torch.where(success, env._traversal_success_streak[env_ids] + 1, 0)
        promote = streak >= successes_to_promote
        env._traversal_success_streak[env_ids] = torch.where(promote, 0, streak)
        move_up = promote & (terrain.terrain_levels[env_ids] < terrain.max_terrain_level - 1)
        terrain.update_env_origins(env_ids, move_up, ~success)
        env.scene.env_origins[env_ids] = terrain.env_origins[env_ids]
    return terrain.terrain_levels.float().mean()


def traversal_outcomes(env, env_ids, success_command_name: str | None = None,
                       success_attribute: str = 'patrol_success'):
    """Accumulate top-row outcomes BEFORE curriculum changes row assignments.

    Cumulative counters avoid repeated logging of the last reset batch counting
    it as new episodes. Use differences between window endpoints for a rate.
    """
    terrain = env.scene.terrain
    cfg = terrain.cfg.terrain_generator
    ids = torch.arange(env.num_envs, device=env.device) if env_ids is None else torch.arange(env.num_envs, device=env.device)[env_ids]
    ids = ids[env.episode_length_buf[ids] > 0]
    proportions = torch.tensor([sub.proportion for sub in cfg.sub_terrains.values()], device=env.device)
    boundaries = torch.cumsum(proportions / proportions.sum(), dim=0)
    samples = torch.arange(cfg.num_cols, device=env.device) / cfg.num_cols + 0.001
    types = torch.searchsorted(boundaries, samples, right=True)[terrain.terrain_types[ids]]
    completed = (traversal_success(env) if success_command_name is None
                 else getattr(env.command_manager.get_term(success_command_name), success_attribute))
    success = completed[ids] & ~env.termination_manager.terminated[ids]
    top = terrain.terrain_levels[ids] == cfg.num_rows - 1
    if not hasattr(env, '_traversal_outcome_counts'):
        env._traversal_outcome_counts = {}
    stats = env._traversal_outcome_counts
    for index, name in enumerate(cfg.sub_terrains):
        mask = (types == index) & top
        for suffix, value in (('attempts', mask.sum()), ('successes', (mask & success).sum())):
            key = f'{name}_top_{suffix}'
            stats[key] = stats.get(key, torch.zeros((), device=env.device)) + value
    return {key: value.clone().float() for key, value in stats.items()}


def finite_height_scan(env, sensor_cfg: SceneEntityCfg = SceneEntityCfg('height_scanner')):
    sensor = env.scene.sensors[sensor_cfg.name]
    heights = env.scene['robot'].data.root_pos_w[:, 2:3] - sensor.data.ray_hits_w[..., 2] - 0.55
    return torch.nan_to_num(heights, nan=0.0, posinf=1.0, neginf=-1.0).clamp(-1.0, 1.0)
