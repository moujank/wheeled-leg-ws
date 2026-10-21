"""Asset/interface contracts and native CPU PPO check; no physics simulation.

Simulator-only config nodes are represented by containers. Runner schemas and
the configclass decorator are imported from the installed Isaac Lab package.
"""
import ast
from copy import deepcopy
import importlib
import importlib.util
import math
from pathlib import Path
import re
from types import ModuleType, SimpleNamespace
import sys
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import torch
from rsl_rl.runners import OnPolicyRunner

ROOT = Path(__file__).resolve().parents[1]
TASK = ROOT / 'source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg'
URDF = ROOT.parents[1] / 'wheeled_description/wheeled_arm_v2/urdf/wheeled_arm_v2.urdf'


def load_runner_configs():
    """Import native config APIs without importing simulator-only package init code."""
    isaac_path = Path(importlib.util.find_spec('isaaclab').submodule_search_locations[0])
    rl_path = Path(importlib.util.find_spec('isaaclab_rl').submodule_search_locations[0]) / 'rsl_rl'
    utils, rl = ModuleType('isaaclab.utils'), ModuleType('isaaclab_rl.rsl_rl')
    utils.__path__, rl.__path__ = [str(isaac_path / 'utils')], [str(rl_path)]
    with patch.dict(sys.modules, {'isaaclab.utils': utils, 'isaaclab_rl.rsl_rl': rl}):
        decorator = importlib.import_module('isaaclab.utils.configclass').configclass
        utils.configclass = decorator
        for name in ('rl_cfg', 'distillation_cfg', 'symmetry_cfg'):
            module = importlib.import_module(f'isaaclab_rl.rsl_rl.{name}')
            for key, value in vars(module).items():
                if key.startswith('RslRl'):
                    setattr(rl, key, value)
        spec = importlib.util.spec_from_file_location('arm_test_runner_cfg', TASK / 'agents/rsl_rl_ppo_cfg.py')
        configs = importlib.util.module_from_spec(spec)
        # dataclasses resolve annotated fields through the defining module.
        sys.modules[spec.name] = configs
        spec.loader.exec_module(configs)
    return configs, decorator


CONFIGS, configclass = load_runner_configs()


class Node(SimpleNamespace):
    def __init__(self, name=None, **kwargs):
        if name is not None:
            kwargs['name'] = name
        super().__init__(**kwargs)

    def replace(self, **kwargs):
        result = deepcopy(self)
        result.__dict__.update(kwargs)
        return result


Node.InitialStateCfg = Node
Node.Ranges = Node


@configclass
class SceneBase:
    num_envs: int = 1
    env_spacing: float = 2.5


@configclass
class EnvBase:
    sim: Node = Node(physx=Node())


@configclass
class ObsBase:
    enable_corruption: bool = False
    concatenate_terms: bool = True


class MdpNamespace:
    def __getattr__(self, name):
        return Node if name.endswith('Cfg') else name


def load_environment():
    scope = {'math': math, 'Path': Path, 'configclass': configclass,
             'ArticulationCfg': Node, 'ImplicitActuatorCfg': Node,
             'sim_utils': SimpleNamespace(**{name: Node for name in (
                 'UsdFileCfg', 'RigidBodyPropertiesCfg', 'ArticulationRootPropertiesCfg',
                 'RigidBodyMaterialCfg', 'MdlFileCfg', 'DomeLightCfg')}),
             '__file__': str(TASK / 'asset/wheeled_arm_v2.py')}
    # Execute actual config bodies; only simulator imports are omitted.
    for path in (TASK / 'asset/wheeled_arm_v2.py', TASK / 'v2_wheeled_arm_v1.py'):
        tree = ast.parse(path.read_text())
        tree.body = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
        scope.update({'InteractiveSceneCfg': SceneBase, 'ManagerBasedRLEnvCfg': EnvBase,
                      'AssetBaseCfg': Node, 'ContactSensorCfg': Node, 'TerrainImporterCfg': Node,
                      'EventTerm': Node, 'ObsGroup': ObsBase, 'ObsTerm': Node, 'RewTerm': Node,
                      'DoneTerm': Node, 'SceneEntityCfg': Node, 'Unoise': Node, 'mdp': MdpNamespace(),
                      'ISAAC_NUCLEUS_DIR': '', 'ISAACLAB_NUCLEUS_DIR': ''})
        exec(compile(tree, str(path), 'exec'), scope)
    return SimpleNamespace(**scope)


def select(patterns, names):
    patterns = [patterns] if isinstance(patterns, str) else patterns
    return [name for name in names if any(re.fullmatch(p, name) for p in patterns)]


class WheeledArmTests(unittest.TestCase):
    def setUp(self):
        self.module = load_environment()
        self.cfg = self.module.WheeledArmV1EnvCfg()
        self.joints = {j.attrib['name']: j for j in ET.parse(URDF).getroot().findall('joint')
                       if j.attrib['type'] != 'fixed'}
        self.names = list(self.joints)

    def test_all_actual_joints_have_one_default_actuator_action_and_reset(self):
        robot = self.cfg.scene.robot
        actions = vars(self.cfg.actions).values()
        resets = (self.cfg.events.reset_leg_joints, self.cfg.events.reset_arm_joints)
        for name, joint in self.joints.items():
            with self.subTest(joint=name):
                defaults = [value for pattern, value in robot.init_state.joint_pos.items()
                            if re.fullmatch(pattern, name)]
                actuators = [a for a in robot.actuators.values()
                             if name in select(a.joint_names_expr, self.names)]
                targets = [a for a in actions if name in select(a.joint_names, self.names)]
                self.assertEqual(len(defaults), 1)
                self.assertEqual(len(actuators), 1)
                self.assertEqual(len(targets), 1)
                self.assertEqual(sum(name in select(r.params['asset_cfg'].joint_names, self.names)
                                     for r in resets), 1)
                actuator, action = actuators[0], targets[0]
                limit = joint.find('limit')
                self.assertGreater(actuator.effort_limit_sim, 0)
                self.assertLessEqual(actuator.effort_limit_sim, float(limit.attrib['effort']))
                self.assertGreater(actuator.velocity_limit_sim, 0)
                clips = [v for p, v in action.clip.items() if re.fullmatch(p, name)]
                self.assertEqual(len(clips), 1)
                lo, hi = clips[0]
                self.assertLess(lo, hi)
                self.assertLessEqual(lo, defaults[0])
                self.assertGreaterEqual(hi, defaults[0])
                if joint.attrib['type'] == 'revolute':
                    lower, upper = (float(limit.attrib[k]) for k in ('lower', 'upper'))
                    middle, half = (upper + lower) / 2, (upper - lower) / 2
                    self.assertGreaterEqual(lo, middle - half * robot.soft_joint_pos_limit_factor)
                    self.assertLessEqual(hi, middle + half * robot.soft_joint_pos_limit_factor)
                else:
                    self.assertLessEqual(max(abs(lo), abs(hi)), actuator.velocity_limit_sim)
        for pattern in robot.init_state.joint_pos:
            self.assertTrue(select(pattern, self.names), f'unmatched default: {pattern}')
        self.assertEqual(len(self.names), 12)
        self.assertEqual(len(select('Arm.*', self.names)), 6)
        self.assertTrue(Path(robot.spawn.usd_path).is_file())

    def widths(self):
        cfg = self.cfg
        action_width = sum(len(select(a.joint_names, self.names)) for a in vars(cfg.actions).values())
        pos_width = len(select(cfg.observations.policy.joint_pos.params['asset_cfg'].joint_names, self.names))
        actor_width = 3 + 3 + 3 + 3 + pos_width + len(self.names) * 2 + action_width
        body_count = len(ET.parse(URDF).getroot().findall('link'))
        return actor_width, actor_width + body_count, action_width

    def test_standalone_play_and_train_configs_do_not_share_mutable_state(self):
        play = self.module.WheeledArmV1EnvCfg_PLAY()
        tree = ast.parse((TASK / 'v2_wheeled_arm_v1.py').read_text())
        for cls in [n for n in tree.body if isinstance(n, ast.ClassDef) and 'EnvCfg' in n.name]:
            self.assertEqual([n.id for n in cls.bases], ['ManagerBasedRLEnvCfg'])
        self.assertTrue(self.cfg.observations.policy.enable_corruption)
        self.assertFalse(play.observations.policy.enable_corruption)
        self.assertFalse(self.cfg.observations.critic.enable_corruption)
        self.assertTrue(play.torque_logging_enabled)
        self.assertIsNone(play.events.add_base_mass)
        self.assertIsNotNone(self.cfg.events.add_base_mass)
        self.assertEqual(play.events.reset_arm_joints.params['position_range'], (0., 0.))
        self.assertNotEqual(self.cfg.events.reset_arm_joints.params['position_range'], (0., 0.))
        play.scene.robot.init_state.joint_pos['Arm_Y1'] = 0.
        self.assertNotEqual(self.cfg.scene.robot.init_state.joint_pos['Arm_Y1'], 0.)
        self.assertEqual(self.widths(), (58, 71, 12))

    def test_arm_regularizer_selects_only_appended_arm_action_group(self):
        tree = ast.parse((TASK / 'mdp/rewards.py').read_text())
        fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'action_rate_l2_selected')
        scope = {'torch': torch, 'ManagerBasedRLEnv': object}
        exec(compile(ast.Module(body=[fn], type_ignores=[]), 'selected_reward', 'exec'), scope)
        manager = Node(active_terms=list(vars(self.cfg.actions)), action_term_dim=[4, 2, 6],
                       action=torch.ones(2, 12), prev_action=torch.zeros(2, 12))
        reward = scope[fn.name]
        env = Node(action_manager=manager)
        self.assertEqual(reward(env, **self.cfg.rewards.arm_action_rate.params).tolist(), [6., 6.])
        manager.action[:, :6] = 100.
        self.assertEqual(reward(env, **self.cfg.rewards.arm_action_rate.params).tolist(), [6., 6.])
        manager.action[:, 6:] = 0.
        self.assertFalse(reward(env, **self.cfg.rewards.arm_action_rate.params).any())

    def test_native_ppo_updates_arm_outputs_with_privileged_critic(self):
        torch.manual_seed(9)
        actor_width, critic_width, num_actions = self.widths()

        class VectorEnv:
            num_envs, max_episode_length, device = 8, 100, 'cpu'

            def __init__(self):
                self.num_actions = num_actions
                self.episode_length_buf = torch.zeros(self.num_envs, dtype=torch.long)
                self.obs = torch.randn(self.num_envs, actor_width)
                self.critic = torch.randn(self.num_envs, critic_width)

            def get_observations(self):
                return self.obs.clone(), {'observations': {'critic': self.critic.clone()}}

            def step(self, actions):
                self.obs += .01
                self.critic -= .02
                obs, info = self.get_observations()
                return obs, -actions.square().sum(-1), torch.zeros(self.num_envs), info

        cfg = CONFIGS.WheeledArmV1PPORunnerCfg()
        cfg.validate()
        self.assertIsNone(cfg.algorithm.symmetry_cfg)
        runner = OnPolicyRunner(VectorEnv(), cfg.to_dict(), device='cpu')
        runner.disable_logs, runner.logger_type = True, 'tensorboard'
        before = runner.alg.policy.actor[-1].weight[6:].detach().clone()
        runner.learn(num_learning_iterations=1)
        after = runner.alg.policy.actor[-1].weight[6:].detach()
        self.assertFalse(torch.equal(before, after))
        self.assertTrue(all(torch.isfinite(p).all() for p in runner.alg.policy.parameters()))
        self.assertEqual(runner.obs_normalizer.mean.shape[-1], actor_width)
        self.assertEqual(runner.privileged_obs_normalizer.mean.shape[-1], critic_width)

    def test_task_ids_resolve_to_existing_config_and_runner_classes(self):
        tree = ast.parse((TASK / '__init__.py').read_text())
        nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_register_rsl_rl_task']
        nodes += [n for n in tree.body if isinstance(n, ast.Expr) and isinstance(n.value, ast.Call)
                  and getattr(n.value.func, 'id', '') == '_register_rsl_rl_task'
                  and n.value.args[0].value in ('Wheeled-Arm-v1', 'Wheeled-Arm-Play-v1')]
        entries = []
        scope = {'gym': Node(register=lambda **kw: entries.append(kw)), '__name__': 'tasks',
                 'agents': Node(__name__='agents'), '_RSL_RL_CFG_ENTRY_POINT': ''}
        exec(compile(ast.Module(body=nodes, type_ignores=[]), 'registry', 'exec'), scope)
        self.assertEqual({e['id'] for e in entries}, {'Wheeled-Arm-v1', 'Wheeled-Arm-Play-v1'})
        for entry in entries:
            self.assertTrue(entry['kwargs']['env_cfg_entry_point'].split(':')[0].endswith('v2_wheeled_arm_v1'))
            getattr(self.module, entry['kwargs']['env_cfg_entry_point'].split(':')[-1])()
            getattr(CONFIGS, entry['kwargs']['rsl_rl_cfg_entry_point'].split(':')[-1])().validate()


if __name__ == '__main__':
    unittest.main()
