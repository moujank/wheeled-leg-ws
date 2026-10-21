"""Deterministic, first-episode transfer and fixed-row evaluation (no PPO updates).

Overrides affect this process only, never the v1 source configuration. Historical
28-input v1 policies omit the newly added effort term explicitly. Results count
one episode per environment to avoid over-weighting policies that fall quickly.
"""
import argparse
from pathlib import Path
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--checkpoint', required=True)
parser.add_argument('--output', required=True)
parser.add_argument('--scene', choices=['v1', 'terrain-flat', 'terrain'], default='terrain-flat')
parser.add_argument('--leg-clips', choices=['v1', 'terrain'], default='terrain')
parser.add_argument('--level', type=int, default=0)
parser.add_argument('--speed', type=float, default=.25)
parser.add_argument('--num_envs', type=int, default=64)
parser.add_argument('--seed', type=int, default=42)
parser.add_argument('--terrain-seed', type=int, default=43, help='Independent terrain geometry seed for held-out maps.')
parser.add_argument('--normalizer-updates', action='store_true', help='Reproduce warm-start count=10000 adaptation without gradients.')
parser.add_argument('--preserve-count', action='store_true', help='Keep pretrained normalization history, as in the corrected terrain warm start.')
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
launcher = AppLauncher(args)

import json
import torch
import gymnasium as gym
import isaaclab.terrains as terrain_gen
import wheeled_leg.tasks
from isaaclab_tasks.utils import parse_env_cfg
from isaaclab.managers import CurriculumTermCfg
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from rsl_rl.runners import OnPolicyRunner
from wheeled_leg.tasks.manager_based.wheeled_leg.agents.rsl_rl_ppo_cfg import WheeledLegTerrainsPPORunnerCfg
from wheeled_leg.tasks.manager_based.wheeled_leg.mdp.curriculums import terrain_levels_forward_traversal
from wheeled_leg.tasks.manager_based.wheeled_leg.mdp.symmetry import install_symmetry_normalization
from warm_start import warm_start_runner
from terrain_evaluation import configure_terrain_evaluation, set_fixed_terrain_level


def main():
    saved = torch.load(args.checkpoint, map_location='cpu', weights_only=False)
    width = saved['model_state_dict']['actor.0.weight'].shape[1]
    task = 'Wheeled-Leg-v1' if args.scene == 'v1' else 'Wheeled-Leg-Terrains'
    cfg = parse_env_cfg(task, device=args.device, num_envs=args.num_envs)
    cfg.seed = args.seed
    cfg.scene.terrain.visual_material = None
    cfg.scene.sky_light.spawn.texture_file = None
    cfg.observations.policy.enable_corruption = False
    cfg.commands.base_velocity.debug_vis = False
    cfg.episode_length_s = 20.
    if args.scene == 'v1':
        if width == 28:
            cfg.observations.policy.joint_effort = None
            cfg.observations.critic.joint_effort = None
        elif width != 34:
            raise ValueError('v1 probe accepts only 28/34 input checkpoints')
        cmd = cfg.commands.base_velocity
        cmd.rel_standing_envs = 0.
        cmd.resampling_time_range = (25., 30.)
        cmd.ranges.lin_vel_x = (args.speed, args.speed)
        cmd.ranges.ang_vel_z = (0., 0.)
    else:
        configure_terrain_evaluation(cfg, args.level, args.speed)
        cfg.scene.terrain.terrain_generator.seed = args.terrain_seed
        cfg.curriculum.terrain_levels = CurriculumTermCfg(
            func=terrain_levels_forward_traversal, params={'update_levels': False})
        if args.scene == 'terrain-flat':
            cfg.scene.terrain.terrain_generator.sub_terrains = {'flat': terrain_gen.MeshPlaneTerrainCfg(proportion=1.)}
            cfg.scene.terrain.terrain_generator.num_cols = 8
    if args.leg_clips == 'v1':
        cfg.actions.joint_pos.clip = {'HipL': (.3, 1.), 'HipR': (-1., -.3),
                                     'KneeL': (.9, 1.7), 'KneeR': (-1.7, -.9)}
    else:
        cfg.actions.joint_pos.clip = {'HipL': (.15, 1.25), 'HipR': (-1.25, -.15),
                                     'KneeL': (.4, 2.2), 'KneeR': (-2.2, -.4)}
    env = gym.make(task, cfg=cfg)
    base = env.unwrapped
    if args.scene != 'v1':
        set_fixed_terrain_level(base, args.level)
    wrapped = RslRlVecEnvWrapper(env)
    agent = WheeledLegTerrainsPPORunnerCfg().to_dict()
    runner = OnPolicyRunner(wrapped, agent, log_dir=None, device=args.device)
    target_width = base.observation_manager.group_obs_dim['policy'][0]
    warm_start_runner(runner, args.checkpoint, target_width - width, wrapped,
                      preserve_normalizer_count=args.preserve_count)
    # Validate physical-input equivalence on the initial observations, including
    # the actual observation manager term mapping, before any normalizer updates.
    obs, _ = wrapped.get_observations()
    actor = runner.alg.policy.actor
    source_actor = torch.nn.Sequential(
        torch.nn.Linear(width, 128), torch.nn.ELU(), torch.nn.Linear(128, 128), torch.nn.ELU(),
        torch.nn.Linear(128, 64), torch.nn.ELU(), torch.nn.Linear(64, 6)).to(args.device)
    source_actor.load_state_dict({k.removeprefix('actor.'): v for k, v in saved['model_state_dict'].items() if k.startswith('actor.')})
    norm = saved['obs_norm_state_dict']
    source_obs = obs[:, :width]
    with torch.no_grad():
        expected = source_actor((source_obs - norm['_mean'].to(args.device)) / (norm['_std'].to(args.device) + runner.obs_normalizer.eps))
        runner.obs_normalizer.eval()
        equivalence = (actor(runner.obs_normalizer(obs)) - expected).abs().max().item()
    install_symmetry_normalization(runner, wrapped)
    runner.obs_normalizer.eval()
    if args.normalizer_updates:
        runner.obs_normalizer.train()
    done_once = torch.zeros(args.num_envs, device=args.device, dtype=torch.bool)
    first_steps = torch.zeros_like(done_once, dtype=torch.long)
    failures = torch.zeros_like(done_once)
    successes = torch.zeros_like(done_once)
    error_sum = torch.zeros(args.num_envs, device=args.device)
    active_steps = torch.zeros_like(error_sum)
    raw_samples = []
    term_counts = {name: 0 for name in base.termination_manager.active_terms}
    with torch.inference_mode():
        for step in range(2501):
            active = ~done_once
            if not active.any():
                break
            action = actor(runner.obs_normalizer(obs))
            if step < 32:
                raw_samples.append(obs.detach().cpu())
            command = base.command_manager.get_command('base_velocity')
            err = torch.linalg.vector_norm(base.scene['robot'].data.root_lin_vel_b[:, :2] - command[:, :2], dim=1)
            error_sum += active * err
            active_steps += active
            obs, _, done, _ = wrapped.step(action)
            first = active & done.bool()
            for name in term_counts:
                term_counts[name] += int((first & base.termination_manager.get_term(name)).sum())
            first_steps[first] = step + 1
            failures[first] = base.termination_manager.terminated[first]
            if hasattr(base, '_forward_last_success'):
                successes[first] = base._forward_last_success[first]
            done_once |= first
    result = dict(checkpoint=str(Path(args.checkpoint).resolve()), scene=args.scene,
                  leg_clips=args.leg_clips, seed=args.seed, terrain_seed=args.terrain_seed,
                  level=args.level, speed=args.speed,
                  normalizer_updates=args.normalizer_updates, num_envs=args.num_envs,
                  preserve_count=args.preserve_count,
                  completed=int(done_once.sum()), failure_fraction=float(failures.float().mean()),
                  mean_first_episode_s=float(first_steps.float().mean()) * base.step_dt,
                  tracking_error_mps=float((error_sum / active_steps.clamp(min=1)).mean()),
                  zero_update_action_max_error=equivalence, termination_counts=term_counts,
                  normalizer_count=float(runner.obs_normalizer.count))
    if hasattr(base, '_forward_terrain_counts'):
        generator = cfg.scene.terrain.terrain_generator
        proportions = torch.tensor([v.proportion for v in generator.sub_terrains.values()], device=args.device)
        boundaries = proportions.cumsum(0) / proportions.sum()
        column_samples = torch.arange(generator.num_cols, device=args.device) / generator.num_cols + .001
        types = torch.searchsorted(boundaries, column_samples, right=True)[base.scene.terrain.terrain_types]
        result['first_episode_by_type'] = {}
        for i, name in enumerate(generator.sub_terrains):
            mask = types == i
            result['first_episode_by_type'][name] = dict(episodes=int(mask.sum()),
                completed=int(done_once[mask].sum()), successes=int(successes[mask].sum()),
                falls=int(failures[mask].sum()))
        result['traversal_counts_all_completed'] = dict(zip(cfg.scene.terrain.terrain_generator.sub_terrains,
                                                          base._forward_terrain_counts.cpu().tolist()))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2))
    torch.save(torch.cat(raw_samples), output.with_suffix('.observations.pt'))
    print('PROBE_RESULT', json.dumps(result))
    wrapped.close()


if __name__ == '__main__':
    try:
        main()
    finally:
        launcher.app.close()
