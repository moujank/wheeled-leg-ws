"""CPU checks for jump credit and compatible checkpoint transfer (no simulator needed)."""
import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace
import sys
import tempfile
import unittest
from unittest.mock import patch

import torch
from rsl_rl.modules.normalizer import EmpiricalNormalization

ROOT = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Entity:
    def __init__(self, name, body_ids=slice(None), joint_ids=slice(None)):
        self.name, self.body_ids = name, body_ids
        self.joint_ids = joint_ids


# Load actual tensor state machine without starting Isaac Sim.
class CommandStub:
    def __init__(self, cfg, env):
        self.cfg, self._env = cfg, env
        self.num_envs, self.device = env.num_envs, env.device


manager_stub = ModuleType('isaaclab.managers')
manager_stub.SceneEntityCfg = Entity
manager_stub.CommandTerm = CommandStub
manager_stub.CommandTermCfg = object
utils_stub = ModuleType('isaaclab.utils')
utils_stub.configclass = lambda cls: cls
with patch.dict(sys.modules, {'isaaclab.managers': manager_stub, 'isaaclab.utils': utils_stub}):
    jump = load_module('jump_terms', ROOT / 'source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg/mdp/jump.py')
    traversal = load_module('traversal_terms', ROOT / 'source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg/mdp/traversal.py')
warm = load_module('warm_start', ROOT / 'scripts/rsl_rl/warm_start.py')


class Scene(dict):
    pass


def jump_env(stage=0):
    origins = torch.tensor([[0., 0., 0.], [8., 0., 2.]])
    data = SimpleNamespace(
        root_pos_w=origins + torch.tensor([0., 0., .55]),
        root_quat_w=torch.tensor([[1., 0., 0., 0.]]).repeat(2, 1),
        body_pos_w=origins[:, None, :] + torch.tensor([[0., -.2, .0935], [0., .2, .0935]]),
        projected_gravity_b=torch.tensor([[0., 0., -1.]]).repeat(2, 1),
        root_lin_vel_w=torch.zeros(2, 3),
    )
    sensor = SimpleNamespace(data=SimpleNamespace(
        current_air_time=torch.zeros(2, 7), current_contact_time=torch.full((2, 7), .1),
        net_forces_w=torch.zeros(2, 7, 3)),
        find_bodies=lambda pattern: ([0, 1], []) if pattern == 'Hub.*' else ([2, 3, 4, 5, 6], []))
    obstacle = SimpleNamespace(data=SimpleNamespace(default_root_state=torch.zeros(2, 13)))
    obstacle.write_root_pose_to_sim = lambda pose, env_ids: setattr(obstacle, 'pose', pose.clone())
    obstacle.write_root_velocity_to_sim = lambda velocity, env_ids: None
    scene = Scene(robot=SimpleNamespace(data=data, find_bodies=lambda _: ([0, 1], [])), jump_barrier=obstacle)
    scene.env_origins, scene.sensors = origins, {'contact_forces': sensor}
    env = SimpleNamespace(num_envs=2, device='cpu', step_dt=.008, scene=scene,
                          episode_length_buf=torch.zeros(2, dtype=torch.long),
                          termination_manager=SimpleNamespace(terminated=torch.zeros(2, dtype=torch.bool)))
    cfg = jump.JumpCourseCommandCfg()
    cfg.initial_stage = stage
    term = jump.JumpCourseCommand(cfg, env)
    env.command_manager = SimpleNamespace(get_term=lambda _: term)
    term._resample_command(slice(None))
    return env, term


def frame(env, term, x=0., bottom=0., air=False, y=0., step=None):
    env.episode_length_buf[:] = env.episode_length_buf + 1 if step is None else step
    origins = env.scene.env_origins
    term.robot.data.root_pos_w[:] = origins + torch.tensor([x, y, .55 + bottom])
    term.robot.data.body_pos_w[:] = origins[:, None, :] + torch.tensor([[x, y-.2, bottom+.0935], [x, y+.2, bottom+.0935]])
    term.sensor.data.current_air_time[:, :2] = .05 if air else 0.
    term.sensor.data.current_contact_time[:, :2] = 0. if air else .1
    term.update_state()


def launch(env, term, x=0., bottom=.12):
    for _ in range(14):
        frame(env, term, x=x)
    if term.moving.any():
        term.trigger_time[:] = 1.224 - term.cfg.crouch_s
    frame(env, term, x=x, bottom=bottom, air=True, step=153)
    for _ in range(12):
        frame(env, term, x=x, bottom=bottom, air=True)


class JumpTests(unittest.TestCase):
    def test_stationary_success_requires_flight_then_stable_landing(self):
        env, term = jump_env(2)
        launch(env, term)
        self.assertFalse(term.successful.any())
        frame(env, term)
        self.assertFalse(term.jump_landed.any())
        for _ in range(16):
            frame(env, term)
        self.assertTrue(term.jump_landed.all())
        self.assertTrue(term.successful.all())
        self.assertTrue(torch.allclose(jump.jump_course_observation(env)[0], jump.jump_course_observation(env)[1], atol=1e-5))

    def test_standing_or_single_wheel_air_is_not_jump(self):
        env, term = jump_env()
        for _ in range(20):
            frame(env, term)
        frame(env, term, bottom=.2, air=True, step=153)
        # Re-run this frame with only one wheel airborne, before accepting it.
        term.in_flight[:] = False
        term.armed[:] = True
        term.last_step[:] = -1
        term.sensor.data.current_air_time[:, 1] = 0
        term.update_state()
        self.assertFalse(term.in_flight.any())
        self.assertFalse(jump.jump_success(env).any())

    def test_strict_moving_crossing_and_goal(self):
        env, term = jump_env(8)
        launch(env, term, x=1.02, bottom=.36)
        frame(env, term, x=1.2, bottom=.36, air=True)
        self.assertFalse(term.crossed.any())
        frame(env, term, x=1.46, bottom=.36, air=True)
        self.assertTrue(term.crossed.all())
        for _ in range(17):
            frame(env, term, x=1.7)
        self.assertTrue(term.jump_landed.all())
        self.assertFalse(term.successful.any())
        frame(env, term, x=2.6)
        self.assertTrue(term.successful.all())
        self.assertTrue(term.metrics['strict_success'].all())
        self.assertTrue(term.success_pulse.all())
        # Same-step calls preserve the pulse, without adding time again.
        previous = term.flight_time.clone()
        self.assertTrue(jump.jump_success_reward(env).all())
        self.assertTrue(torch.equal(previous, term.flight_time))
        frame(env, term, x=2.6)
        self.assertFalse(term.success_pulse.any())

    def test_walking_over_barrier_never_counts(self):
        env, term = jump_env(8)
        frame(env, term, x=1.5, bottom=.3, step=160)
        self.assertTrue(jump.jump_failed_crossing(env).all())
        self.assertFalse(term.crossed.any())

    def test_low_crossing_side_bypass_and_leg_contact_rejected(self):
        for failure in ('low', 'side', 'leg'):
            with self.subTest(failure=failure):
                env, term = jump_env(8)
                launch(env, term, x=1.02, bottom=.36)
                if failure == 'leg':
                    term.sensor.data.net_forces_w[:, 2, 2] = 2
                frame(env, term, x=1.2, bottom=.1 if failure == 'low' else .36,
                      y=.6 if failure == 'side' else 0., air=True)
                frame(env, term, x=1.46, bottom=.36, air=True)
                self.assertTrue(term.invalid.all())
                self.assertFalse(term.crossed.any())

    def test_two_separate_hops_cannot_be_combined(self):
        env, term = jump_env(8)
        launch(env, term, x=1.02, bottom=.36)
        frame(env, term, x=1.2, bottom=.36, air=True)
        frame(env, term, x=1.3, bottom=.36, air=True)
        term.last_step[:] = -1
        term.sensor.data.current_air_time[:, 0] = 0
        term.update_state()
        frame(env, term, x=1.46, bottom=.36, air=True)
        self.assertFalse(term.crossed.any())
        self.assertTrue(term.invalid.all())

    def test_partial_reset_and_stage_geometry(self):
        env, term = jump_env(8)
        self.assertTrue(torch.allclose(env.scene['jump_barrier'].pose[:, 2] + .1,
                                       env.scene.env_origins[:, 2] + .2))
        self.assertTrue((term.command[:, 0] > 0).all())
        term.crossed[:] = True
        term.level[0] = 0
        term._resample_command(slice(0, 1))
        self.assertFalse(term.crossed[0])
        self.assertTrue(term.crossed[1])
        self.assertEqual(term.command[0, 0], 0)
        self.assertEqual(tuple(jump.jump_course_observation(env).shape), (2, 12))
        self.assertTrue(torch.isfinite(jump.jump_course_observation(env)).all())

    def test_curriculum_promotes_success_but_not_failed_termination(self):
        env, term = jump_env()
        env.episode_length_buf[:] = 200
        term.successful[:] = True
        env.termination_manager.terminated[1] = True
        for _ in range(20):
            jump.jump_course_curriculum(env, slice(None))
        self.assertEqual(term.level.tolist(), [1, 0])
        self.assertEqual(term.attempts.tolist(), [0, 20])

    def test_recovery_penalty_allows_preparation_and_flight(self):
        env, term = jump_env()
        term.robot.data.root_lin_vel_w[:, 2] = 1
        frame(env, term, step=round(1.1 / .008))
        self.assertFalse(jump.jump_recovery_velocity_l2(env).any())
        frame(env, term, step=round(2.5 / .008))
        self.assertTrue(torch.equal(jump.jump_recovery_velocity_l2(env), torch.ones(2)))


class TraversalTests(unittest.TestCase):
    def env(self):
        env, _ = jump_env()
        env.command_manager.get_command = lambda _: torch.tensor([[.5, 0., 0.], [.9, 0., 0.]])
        env.scene.sensors['height_scanner'] = SimpleNamespace(data=SimpleNamespace(ray_hits_w=torch.zeros(2, 6, 3)))
        env.scene['robot'].data.joint_vel = torch.ones(2, 6)
        terrain = SimpleNamespace(terrain_levels=torch.tensor([3, 7]), max_terrain_level=8,
                                  terrain_types=torch.tensor([0, 1]), env_origins=env.scene.env_origins.clone())
        terrain.cfg = SimpleNamespace(terrain_generator=SimpleNamespace(
            num_rows=8, num_cols=2, sub_terrains={name: SimpleNamespace(proportion=.5)
                for name in ('inverted_pyramid', 'random_height_squares')}))
        def update(ids, up, down):
            levels = terrain.terrain_levels[ids] + up.long() - down.long()
            # Reproduce the importer wrap so tests detect accidental top overflow.
            terrain.terrain_levels[ids] = torch.where(levels >= 8, 0, levels.clamp(min=0))
        terrain.update_env_origins = update
        env.scene.terrain = terrain
        return env

    def test_progress_caps_at_command_and_penalizes_reverse(self):
        env = self.env()
        env.scene['robot'].data.root_lin_vel_w[:, 0] = 2
        self.assertTrue(torch.allclose(traversal.traversal_progress(env), torch.tensor([.5, .9])))
        env.scene['robot'].data.root_lin_vel_w[:, 0] = -2
        self.assertTrue(torch.equal(traversal.traversal_progress(env), -torch.ones(2)))

    def test_flat_cost_relaxes_at_obstacle_and_is_height_invariant(self):
        env = self.env()
        env.scene['robot'].data.root_lin_vel_w[:, 2] = 1
        hits = env.scene.sensors['height_scanner'].data.ray_hits_w
        hits[1, :, 2] = 10
        self.assertTrue(torch.equal(traversal.traversal_flat_weight(env), torch.ones(2)))
        hits[1, 0, 2] += .2
        self.assertAlmostEqual(traversal.traversal_flat_vertical_velocity(env)[0].item(), .85**2, places=5)
        self.assertEqual(traversal.traversal_flat_vertical_velocity(env)[1].item(), 0)
        self.assertEqual(traversal.traversal_flat_leg_motion(env, Entity('robot', joint_ids=[0, 2, 4, 5]))[0].item(), 4)
        hits[0, 0, 2] = float('nan')
        self.assertFalse(traversal.traversal_flat_weight(env).any())

    def test_curriculum_ignores_initial_reset_and_counts_early_failure(self):
        env = self.env()
        traversal.traversal_curriculum(env, None)
        self.assertEqual(env.scene.terrain.terrain_levels.tolist(), [3, 7])
        env.episode_length_buf[0] = 1
        traversal.traversal_curriculum(env, [0])
        self.assertEqual(env.scene.terrain.terrain_levels.tolist(), [2, 7])

    def test_two_successes_promote_and_top_never_wraps(self):
        env = self.env()
        env.episode_length_buf[:] = 200
        env.scene['robot'].data.root_pos_w[:, 0] += 3
        traversal.traversal_curriculum(env, slice(None))
        self.assertEqual(env.scene.terrain.terrain_levels.tolist(), [3, 7])
        traversal.traversal_curriculum(env, slice(None))
        self.assertEqual(env.scene.terrain.terrain_levels.tolist(), [4, 7])
        env.termination_manager.terminated[0] = True
        traversal.traversal_curriculum(env, [0])
        self.assertEqual(env.scene.terrain.terrain_levels.tolist(), [3, 7])
        self.assertEqual(env._traversal_success_streak.tolist(), [0, 0])

    def test_top_outcomes_count_failures_without_empty_batch_bias(self):
        env = self.env()
        self.assertTrue(all(v == 0 for v in traversal.traversal_outcomes(env, None).values()))
        env.episode_length_buf[:] = 200
        env.scene['robot'].data.root_pos_w[:, 0] += 3
        stats = traversal.traversal_outcomes(env, slice(None))
        self.assertEqual(stats['random_height_squares_top_attempts'], 1)
        self.assertEqual(stats['random_height_squares_top_successes'], 1)
        self.assertEqual(stats['inverted_pyramid_top_attempts'], 0)
        env.termination_manager.terminated[1] = True
        stats = traversal.traversal_outcomes(env, [1])
        self.assertEqual(stats['random_height_squares_top_attempts'], 2)
        self.assertEqual(stats['random_height_squares_top_successes'], 1)


class Policy(torch.nn.Module):
    def __init__(self, obs, critic):
        super().__init__()
        self.actor = torch.nn.Sequential(torch.nn.Linear(obs, 8), torch.nn.ELU(), torch.nn.Linear(8, 6))
        self.critic = torch.nn.Sequential(torch.nn.Linear(critic, 8), torch.nn.ELU(), torch.nn.Linear(8, 1))
        self.log_std = torch.nn.Parameter(torch.zeros(6))


def runner(obs, critic):
    policy = Policy(obs, critic)
    return SimpleNamespace(device='cpu', empirical_normalization=True,
                           obs_normalizer=EmpiricalNormalization([obs]),
                           privileged_obs_normalizer=EmpiricalNormalization([critic]),
                           alg=SimpleNamespace(policy=policy, optimizer=torch.optim.Adam(policy.parameters(), lr=.00025)),
                           current_learning_iteration=0)


class TransferTests(unittest.TestCase):
    def transfer(self, appended):
        source, target = runner(28, 35), runner(28+appended, 35+appended)
        source.obs_normalizer.update(torch.randn(50, 28))
        source.privileged_obs_normalizer.update(torch.randn(50, 35))
        saved = dict(model_state_dict=source.alg.policy.state_dict(),
                     obs_norm_state_dict=source.obs_normalizer.state_dict(),
                     privileged_obs_norm_state_dict=source.privileged_obs_normalizer.state_dict(), iter=5000)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'model.pt'
            torch.save(saved, path)
            warm.warm_start_runner(target, path, appended)
        for norm in (source.obs_normalizer, target.obs_normalizer, source.privileged_obs_normalizer,
                     target.privileged_obs_normalizer):
            norm.eval()
        obs, priv = torch.randn(10, 28), torch.randn(10, 35)
        self.assertTrue(torch.allclose(source.alg.policy.actor(source.obs_normalizer(obs)),
                        target.alg.policy.actor(target.obs_normalizer(torch.cat((obs, torch.randn(10, appended)), 1))), atol=1e-6))
        self.assertTrue(torch.allclose(source.alg.policy.critic(source.privileged_obs_normalizer(priv)),
                        target.alg.policy.critic(target.privileged_obs_normalizer(torch.cat((priv, torch.randn(10, appended)), 1))), atol=1e-6))
        self.assertEqual(target.current_learning_iteration, 0)
        self.assertEqual(target.alg.optimizer.param_groups[0]['lr'], .00025)
        self.assertFalse(target.alg.optimizer.state)

    def test_same_observations(self):
        self.transfer(0)

    def test_appended_jump_observations_preserve_initial_policy(self):
        self.transfer(12)

    def test_appended_terrain_scan_preserves_initial_policy(self):
        self.transfer(54)

    def test_reject_incompatible_and_nonfinite(self):
        with self.assertRaises(ValueError):
            warm._expand_state({'actor.0.weight': torch.zeros(8, 32)},
                               {'actor.0.weight': torch.zeros(8, 35)}, {'actor.0.weight'}, 4)
        with self.assertRaises(ValueError):
            warm._expand_state({'x': torch.tensor([float('nan')])}, {'x': torch.ones(1)}, set(), 0)


if __name__ == '__main__':
    unittest.main()
