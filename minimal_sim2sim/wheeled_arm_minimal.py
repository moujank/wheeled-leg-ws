#!/usr/bin/env python3
"""独立 Wheeled-Arm-Play-v1 sim2sim：58D observation -> checkpoint actor -> 12D action。

不依赖 leg 迁移程序或 Isaac Sim。默认直接读取 best/model.pt 及同次训练 params。
使用标称 PLAY（无噪声/随机化）；XML 中两处修正惯量和 PhysX 速度约束差异会打印。
python wheeled_arm_minimal.py --check-only
python wheeled_arm_minimal.py --no-viewer --no-realtime --duration 10 --log /tmp/arm.jsonl
默认打开 viewer，R 重置；--mode zero 为零动作诊断，不是平衡策略。
"""
from __future__ import annotations

import argparse
import ast
from collections import deque
from contextlib import ExitStack
import hashlib
import json
import math
from pathlib import Path
import sys
import threading
import time

import mujoco
import numpy as np

WORKSPACE = Path(__file__).resolve().parents[3]
TRAIN = WORKSPACE / 'modules/wheeled_train/wheeled_leg'
TASK = TRAIN / 'source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg'
MODEL_XML = WORKSPACE / 'modules/wheeled_mujoco/asset/mjcf/wheeled_arm_v2.xml'
POLICY_PATH = TRAIN / 'logs/rsl_rl/wheeled_arm_v1/best/model.pt'
TASK_SOURCE = TASK / 'v2_wheeled_arm_v1.py'
ASSET_SOURCE = TASK / 'asset/wheeled_arm_v2.py'
SIM_DT, DECIMATION = .002, 4
CONTROL_DT = SIM_DT * DECIMATION
OBS_DIM, ACTION_DIM = 58, 12
# Actual robot.joint_names in the selected run's PLAY torque metadata, NOT MJCF order.
OBS_ORDER = ['Arm_R', 'HipL', 'HipR', 'Arm_Y1', 'KneeL', 'KneeR',
             'Arm_Y2', 'HubL', 'HubR', 'Arm_P1', 'Arm_Y3', 'Arm_P2']
ARM_ORDER = ['Arm_R', 'Arm_Y1', 'Arm_Y2', 'Arm_P1', 'Arm_Y3', 'Arm_P2']
ACTION_ORDER = ['HipL', 'HipR', 'KneeL', 'KneeR', 'HubL', 'HubR'] + ARM_ORDER
POSITION_ORDER = [n for n in OBS_ORDER if not n.startswith('Hub')]
DEFAULT_Q = dict(zip(ACTION_ORDER, [.65, -.65, 1.3, -1.3, 0, 0, 0, -.65, 1.2, 0, 0, 0]))
KP = np.array([20, 20, 30, 30, 0, 0] + [40] * 6, dtype=float)
KD = np.array([1.5] * 6 + [6] * 6)
EFFORT_LIMIT = np.array([120] * 4 + [17] * 2 + [27] * 6)
SPEED_LIMIT = np.array([2 * math.pi] * 6 + [2.] * 6)  # ACTION_ORDER
SCALE = np.array([.25] * 4 + [4.] * 2 + [.15] * 6)
TARGET_LIMIT = np.array([[.3, 1], [-1, -.3], [.9, 1.7], [-1.7, -.9],
                         [-6, 6], [-6, 6], [-2.8, 2.8]] + [[-1.4, 1.4]] * 5)
OBS_SLICES = {'base_lin_vel': (0, 3), 'base_ang_vel': (3, 6), 'projected_gravity': (6, 9),
              'velocity_commands': (9, 12), 'joint_pos': (12, 22), 'joint_vel': (22, 34),
              'joint_effort': (34, 46), 'actions': (46, 58)}
MASS = dict(zip(['base_link'] + ACTION_ORDER,
                [6.382, 3.721, 3.721, .7847, .7847, 1.1563, 1.1563, .49, .958, .738, .632, .637, .5]))
ACTION_SPEC = {
    'joint_pos': dict(joint_names=['Hip.*', 'Knee.*'], scale=.25, use_default_offset=True,
                      clip={'HipL': (.3, 1.), 'HipR': (-1., -.3), 'KneeL': (.9, 1.7), 'KneeR': (-1.7, -.9)}),
    'joint_vel': dict(joint_names=['Hub.*'], scale=4., use_default_offset=True, clip={'Hub.*': (-6., 6.)}),
    'joint_pos_arm': dict(joint_names=ARM_ORDER, preserve_order=True, scale=.15, use_default_offset=True,
                          clip={'Arm_R': (-2.8, 2.8), 'Arm_(Y.*|P.*)': (-1.4, 1.4)}),
}
OBS_SPEC = {
    'base_lin_vel': ('base_lin_vel', (-5., 5.), None),
    'base_ang_vel': ('base_ang_vel', (-10., 10.), .25),
    'projected_gravity': ('projected_gravity', None, None),
    'velocity_commands': ('generated_commands', None, None),
    'joint_pos': ('joint_pos_rel', None, None),
    'joint_vel': ('joint_vel_rel', (-30., 30.), .1),
    'joint_effort': ('joint_effort', (-100., 100.), None),
    'actions': ('last_action', (-5., 5.), None),
}


def require(label, actual, expected):
    if actual != expected:
        raise ValueError(f'{label} 不匹配: actual={actual!r}, expected={expected!r}')


def vector(value, size, label):
    value = np.asarray(value, dtype=np.float32)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError(f'{label} 必须是有限 ({size},) 向量，实际 {value.shape}')
    return value


def read_yaml(path):
    try:
        import yaml
    except ModuleNotFoundError as exc:
        if exc.name != 'yaml':
            raise
        raise SystemExit(
            f'当前 Python 环境缺少 PyYAML（用于读取 checkpoint 的训练配置）。\n'
            f'Python: {sys.executable}\n'
            f'请使用同一个解释器安装：\n  "{sys.executable}" -m pip install PyYAML'
        ) from None
    # Read only data: explicitly handle Isaac's tuple/slice serialization, never execute Python YAML tags.
    class DataLoader(yaml.SafeLoader):
        pass
    DataLoader.add_constructor('tag:yaml.org,2002:python/tuple',
                               lambda loader, node: tuple(loader.construct_sequence(node)))
    DataLoader.add_constructor('tag:yaml.org,2002:python/object/apply:builtins.slice',
                               lambda loader, node: {'slice': loader.construct_sequence(node)})
    return yaml.load(path.read_text(), Loader=DataLoader)


def audit_terms(actions, observations, label):
    require(label + '.action order', list(actions), list(ACTION_SPEC))
    for name, expected in ACTION_SPEC.items():
        for field, value in expected.items():
            require(f'{label}.{name}.{field}', actions[name].get(field), value)
        require(f'{label}.{name}.preserve_order', actions[name].get('preserve_order', False), name == 'joint_pos_arm')
    terms = {k: v for k, v in observations.items() if isinstance(v, dict) and 'func' in v}
    require(label + '.obs order', list(terms), list(OBS_SPEC))
    for name, (func, clip, scale) in OBS_SPEC.items():
        term = terms[name]
        require(f'{label}.{name}.func', term['func'].replace(':', '.').split('.')[-1], func)
        require(f'{label}.{name}.clip', term.get('clip'), clip)
        require(f'{label}.{name}.scale', term.get('scale'), scale)
        require(f'{label}.{name}.history', term.get('history_length', 0), 0)
        require(f'{label}.{name}.modifiers', term.get('modifiers'), None)
    entity = terms['joint_pos']['params']['asset_cfg']
    require(label + '.position selection', entity['joint_names'], ['Hip.*', 'Knee.*', 'Arm.*'])
    require(label + '.position order', entity.get('preserve_order', False), False)
    require(label + '.command', terms['velocity_commands']['params']['command_name'], 'base_velocity')


def audit_sources(verbose=True):
    """Inspect current Python config AST; do not import/execute Isaac configuration."""
    def assignments(node):
        result = {}
        for n in node.body:
            if isinstance(n, ast.Assign):
                for target in n.targets:
                    result[ast.unparse(target)] = n.value
            elif isinstance(n, ast.AnnAssign):
                result[ast.unparse(n.target)] = n.value
        return result

    def cls(node, name):
        return next(n for n in node.body if isinstance(n, ast.ClassDef) and n.name == name)

    def decode(n):
        if isinstance(n, ast.Call):
            if ast.unparse(n.func) == 'math.radians':
                return math.radians(ast.literal_eval(n.args[0]))
            return {k.arg: decode(k.value) for k in n.keywords}
        if isinstance(n, ast.Attribute):
            return ast.unparse(n)
        if isinstance(n, ast.Dict):
            result = {}
            for k, v in zip(n.keys, n.values):
                if k is None:
                    result.update(decode(v))
                else:
                    result[decode(k)] = decode(v)
            return result
        if isinstance(n, ast.Name) and n.id == 'ARM_FOLDED_JOINT_POS':
            return folded
        if ast.unparse(n) == '2.0 * math.pi':
            return 2 * math.pi
        return ast.literal_eval(n)

    task = ast.parse(TASK_SOURCE.read_text())
    asset = ast.parse(ASSET_SOURCE.read_text())
    folded = decode(assignments(asset)['ARM_FOLDED_JOINT_POS'])
    cfg = decode(assignments(asset)['WHEELED_ARM_CFG'])
    audit_asset(cfg, 'current asset')
    actions = {k: decode(v) for k, v in assignments(cls(task, 'ActionsCfg')).items()}
    obs = {k: decode(v) for k, v in assignments(cls(cls(task, 'ObservationsCfg'), 'PolicyCfg')).items()}
    audit_terms(actions, obs, 'current task')
    scene = assignments(cls(task, 'WheeledArmV1SceneCfg'))
    require('task asset', ast.unparse(scene['robot'].func), 'WHEELED_ARM_CFG.replace')
    require('task asset overrides', list(decode(scene['robot'])), ['prim_path'])
    material = decode(next(k.value for k in scene['terrain'].keywords if k.arg == 'physics_material'))
    for field in ('static_friction', 'dynamic_friction'):
        require('terrain.' + field, material[field], 1.)
    require('contact history', decode(scene['contact_forces'])['history_length'], 3)
    commands = decode(assignments(cls(task, 'CommandsCfg'))['base_velocity'])
    require('command resampling', commands['resampling_time_range'], (6., 10.))
    require('heading command', commands['heading_command'], False)
    require('command ranges', commands['ranges'],
            dict(lin_vel_x=(-.5, .5), lin_vel_y=(0., 0.), ang_vel_z=(-.5, .5), heading=None))
    terms = {k: decode(v) for k, v in assignments(cls(task, 'TerminationsCfg')).items()}
    require('tilt threshold', terms['excessive_tilt']['params']['limit_angle'], math.pi/3)
    require('contact threshold', terms['base_contact']['params']['threshold'], 1.)
    require('contact bodies', terms['base_contact']['params']['sensor_cfg']['body_names'],
            ['base_link', 'Hip.*', 'Knee.*', 'Arm.*'])
    for cname in ('WheeledArmV1EnvCfg', 'WheeledArmV1EnvCfg_PLAY'):
        init = next(n for n in cls(task, cname).body if isinstance(n, ast.FunctionDef) and n.name == '__post_init__')
        fields = assignments(init)
        require(cname + '.dt', decode(fields['self.sim.dt']), SIM_DT)
        require(cname + '.decimation', decode(fields['self.decimation']), DECIMATION)
        if cname.endswith('PLAY'):
            require('PLAY episode length', decode(fields['self.episode_length_s']), 40.)
            require('PLAY standing probability', decode(fields['self.commands.base_velocity.rel_standing_envs']), 0.)
            require('PLAY noise', decode(fields['self.observations.policy.enable_corruption']), False)
            for field, expected in [("self.events.reset_base.params['pose_range']", {}),
                                    ("self.events.reset_base.params['velocity_range']", {}),
                                    ("self.events.reset_leg_joints.params['position_range']", (1., 1.)),
                                    ("self.events.reset_arm_joints.params['position_range']", (0., 0.))]:
                require('PLAY reset ' + field, decode(fields[field]), expected)
            for event in ('physics_material', 'add_base_mass', 'base_external_force_torque', 'push_robot'):
                require('PLAY ' + event, decode(fields['self.events.' + event]), None)
    if verbose:
        print('[PASS] 当前训练源码：12关节增益/限幅、折叠姿态、动作/观测配置、dt/decimation、PLAY随机化。')


def audit_asset(cfg, label):
    init = cfg['init_state']
    require(label + '.base', tuple(init['pos']), (0., 0., .58))
    expected = {n: v for n, v in DEFAULT_Q.items() if not n.startswith('Hub')}
    expected['Hub.*'] = 0.
    require(label + '.pose', init['joint_pos'], expected)
    require(label + '.velocity', init['joint_vel'], {'.*': 0.})
    require(label + '.soft limits', cfg['soft_joint_pos_limit_factor'], .9)
    for name, kp, kd, limit, speed in [('hip_act', 20., 1.5, 120., 2*math.pi),
                                      ('knee_act', 30., 1.5, 120., 2*math.pi),
                                      ('hub_act', 0., 1.5, 17., 2*math.pi), ('arm_act', 40., 6., 27., 2.)]:
        for field, expected in zip(('stiffness', 'damping', 'effort_limit_sim', 'velocity_limit_sim'), (kp, kd, limit, speed)):
            require(f'{label}.{name}.{field}', cfg['actuators'][name][field], expected)
    require(label + '.self collision', cfg['spawn']['articulation_props']['enabled_self_collisions'], False)
    for field, expected in [('disable_gravity', False), ('linear_damping', 0.), ('angular_damping', 0.)]:
        require(label + '.' + field, cfg['spawn']['rigid_props'][field], expected)


class Policy:
    """Restore deterministic actor and frozen empirical normalizer directly from RSL-RL checkpoint."""
    def __init__(self, path, params_dir=None):
        import torch
        self.torch = torch
        self.path = Path(path).resolve(strict=True)
        self.params_dir = Path(params_dir) if params_dir else self.path.parent / 'params'
        agent = read_yaml(self.params_dir / 'agent.yaml')
        env = read_yaml(self.params_dir / 'env.yaml')
        require('agent class', agent['policy']['class_name'], 'ActorCritic')
        require('normalization', agent['empirical_normalization'], True)
        require('raw action clipping', agent.get('clip_actions'), None)
        require('actor activation', agent['policy']['activation'], 'elu')
        require('checkpoint dt', env['sim']['dt'], SIM_DT)
        require('checkpoint decimation', env['decimation'], DECIMATION)
        audit_asset(env['scene']['robot'], 'checkpoint asset')
        audit_terms(env['actions'], env['observations']['policy'], 'checkpoint task')
        require('checkpoint concatenation', env['observations']['policy']['concatenate_terms'], True)
        require('checkpoint observation history', env['observations']['policy']['history_length'], None)
        checkpoint = torch.load(self.path, map_location='cpu', weights_only=True)
        dims = [OBS_DIM, *agent['policy']['actor_hidden_dims'], ACTION_DIM]
        layers = []
        for i, (din, dout) in enumerate(zip(dims, dims[1:])):
            layers.append(torch.nn.Linear(din, dout))
            if i < len(dims) - 2:
                layers.append(torch.nn.ELU())
        self.actor = torch.nn.Sequential(*layers)
        weights = {k.removeprefix('actor.'): v for k, v in checkpoint['model_state_dict'].items() if k.startswith('actor.')}
        self.actor.load_state_dict(weights, strict=True)
        self.actor.eval().requires_grad_(False)
        norm = checkpoint['obs_norm_state_dict']
        self.mean, self.std = norm['_mean'], norm['_std']
        for name, value in [('mean', self.mean), ('std', self.std)]:
            if tuple(value.shape) != (1, OBS_DIM) or not torch.isfinite(value).all():
                raise ValueError(f'checkpoint normalizer {name} 必须为有限 (1,58)')
        if (self.std < 0).any() or any(not torch.isfinite(v).all() for v in weights.values()):
            raise ValueError('checkpoint 参数或 normalizer 无效')
        # RSL-RL EmpiricalNormalization default eps=1e-2 (not sqrt(var+eps)).
        self.eps = .01
        self.iteration = checkpoint.get('iter')
        self(vector(np.zeros(OBS_DIM), OBS_DIM, 'probe'))
        print(f'[POLICY] {self.path}\n  actor={dims}, ELU, iteration={self.iteration}; deterministic mean action')
        print('[PASS] checkpoint params 与当前迁移契约一致；归一化 (obs-mean)/(std+0.01)，统计量冻结。')

    def normalize(self, obs):
        x = self.torch.from_numpy(vector(obs, OBS_DIM, 'observation').copy())[None]
        return (x - self.mean) / (self.std + self.eps)

    def __call__(self, obs):
        with self.torch.inference_mode():
            output = self.actor(self.normalize(obs)).numpy()
        if output.shape != (1, ACTION_DIM):
            raise ValueError(f'actor output shape={output.shape}')
        return vector(output[0], ACTION_DIM, 'action')


def check_metadata(checkpoint, explicit=None):
    paths = [Path(explicit)] if explicit else sorted(Path(checkpoint).resolve().parent.glob('play_joint_torque/*/metadata.txt'))
    if not paths:
        raise FileNotFoundError('需要该次训练的 PLAY metadata.txt 确认实际关节顺序；请使用 --isaac-metadata')
    for path in paths:
        entries = dict(line.split('=', 1) for line in path.read_text().splitlines() if '=' in line)
        require(str(path) + '.task', entries.get('task'), 'Wheeled-Arm-Play-v1')
        require(str(path) + '.joint_names', entries['joint_names'].split(','), OBS_ORDER)
        require(str(path) + '.step_dt', float(entries['step_dt']), CONTROL_DT)
    print(f'[PASS] 实际 Isaac 关节顺序/控制周期：{paths[-1]}')


def build_joint_order(model):
    if (model.nq, model.nv, model.nu) != (19, 18, 12):
        raise ValueError('需要浮动基座 + 12关节模型 (nq=19,nv=18,nu=12)')
    state = [model.joint(i).name for i in range(model.njnt) if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
    if len(state) != 12 or set(state) != set(OBS_ORDER):
        raise ValueError(f'关节集合不匹配: {state}')
    if np.any(model.actuator_trntype != mujoco.mjtTrn.mjTRN_JOINT):
        raise ValueError('需要 joint transmission')
    control = [model.joint(int(j)).name for j in model.actuator_trnid[:, 0]]
    if len(set(control)) != 12 or set(control) != set(ACTION_ORDER):
        raise ValueError(f'执行器绑定不匹配: {control}')
    return {'q_obs': np.array([model.joint(n).qposadr[0] for n in OBS_ORDER]),
            'v_obs': np.array([model.joint(n).dofadr[0] for n in OBS_ORDER]),
            'q_action': np.array([model.joint(n).qposadr[0] for n in ACTION_ORDER]),
            'v_action': np.array([model.joint(n).dofadr[0] for n in ACTION_ORDER]),
            'actuator': np.array([control.index(n) for n in ACTION_ORDER]),
            'action_to_obs': np.array([ACTION_ORDER.index(n) for n in OBS_ORDER]),
            'position_indices': np.array([OBS_ORDER.index(n) for n in POSITION_ORDER]),
            'state_names': state, 'control_names': control}


def action_to_targets(action):
    action = vector(action, ACTION_DIM, 'action')
    return np.clip(np.array([DEFAULT_Q[n] for n in ACTION_ORDER]) + SCALE * action,
                   TARGET_LIMIT[:, 0], TARGET_LIMIT[:, 1])


def check_alignment(model, order, verbose=True):
    def check(label, actual, expected):
        if np.shape(actual) != np.shape(expected) or not np.allclose(actual, expected, atol=1e-7, rtol=1e-6):
            raise ValueError(f'[FAIL] {label}: actual={actual}, expected={expected}')
        if verbose:
            print(f'[PASS] {label}: {actual}')
    a = order['actuator']
    j = np.array([model.joint(n).id for n in ACTION_ORDER])
    check('dt / control_dt', [model.opt.timestep, model.opt.timestep*DECIMATION], [SIM_DT, CONTROL_DT])
    check('gravity', model.opt.gravity, [0, 0, -9.81])
    check('integrator implicitfast', int(model.opt.integrator), int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST))
    for field in ('dof_damping', 'dof_armature', 'dof_frictionloss'):
        check(field, getattr(model, field), np.zeros(model.nv))
    check('gear', model.actuator_gear[a], np.tile([1, 0, 0, 0, 0, 0], (12, 1)))
    for field, expected in [('actuator_gaintype', 0), ('actuator_biastype', 1), ('actuator_dyntype', 0),
                            ('actuator_ctrllimited', 1), ('actuator_forcelimited', 1)]:
        check(field, getattr(model, field)[a], np.full(12, expected))
    check('gain', model.actuator_gainprm[a, 0], np.where(KP == 0, KD, KP))
    check('bias', model.actuator_biasprm[a, :3], np.c_[np.zeros(12), -KP, -KD])
    check('target ranges', model.actuator_ctrlrange[a], TARGET_LIMIT)
    check('effort limits', model.actuator_forcerange[a], np.c_[-EFFORT_LIMIT, EFFORT_LIMIT])
    check('joint force limits', model.jnt_actfrcrange[j], np.c_[-EFFORT_LIMIT, EFFORT_LIMIT])
    check('joint force limited', model.jnt_actfrclimited[j], np.ones(12))
    check('physical limited joints', model.jnt_limited[j], [1]*4+[0]*2+[1]*6)
    check('physical ranges', model.jnt_range[j[np.r_[0:4, 6:12]]],
          np.array([[-3.14, 3.14]]*5+[[-1.57, 1.57]]*5))
    check('body masses', [model.body(n).mass[0] for n in MASS], list(MASS.values()))
    for name in ('imu', 'base_com'):
        check(name + ' body', model.site(name).bodyid[0], model.body('base_link').id)
        check(name + ' orientation', model.site(name).quat, [1, 0, 0, 0])
    check('imu position', model.site('imu').pos, [0, 0, 0])
    check('COM site', model.site('base_com').pos, model.body('base_link').ipos)
    for n, kind, site in [('base_com_linvel', mujoco.mjtSensor.mjSENS_VELOCIMETER, 'base_com'),
                          ('imu_gyro', mujoco.mjtSensor.mjSENS_GYRO, 'imu'),
                          ('imu_linear_velocity', mujoco.mjtSensor.mjSENS_VELOCIMETER, 'imu')]:
        s = model.sensor(n)
        check(n + ' binding/type/dim', [s.objid[0], s.type[0], s.dim[0]], [model.site(site).id, int(kind), 3])
    for i, n in enumerate(ACTION_ORDER):
        for suffix, kind, obj in [('pos', mujoco.mjtSensor.mjSENS_JOINTPOS, j[i]),
                                  ('vel', mujoco.mjtSensor.mjSENS_JOINTVEL, j[i]),
                                  ('torque', mujoco.mjtSensor.mjSENS_ACTUATORFRC, a[i])]:
            s = model.sensor(n+'_'+suffix)
            check(n+'_'+suffix, [s.objid[0], s.type[0], s.dim[0]], [obj, int(kind), 1])
    for g in range(model.ngeom):
        if not (model.geom_contype[g] or model.geom_conaffinity[g]):
            continue
        check(f'geom {g} mask/condim', [model.geom_contype[g], model.geom_conaffinity[g], model.geom_condim[g]],
              [1, 2, 3] if model.geom_bodyid[g] == 0 else [2, 1, 3])
        check(f'geom {g} friction', model.geom_friction[g], [1, 0, 0])
    home = model.key('home').id
    check('home base', model.key_qpos[home, :7], [0, 0, .58, 1, 0, 0, 0])
    check('home joints', model.key_qpos[home, order['q_action']], action_to_targets(np.zeros(12)))
    check('home ctrl', model.key_ctrl[home, a], action_to_targets(np.zeros(12)))
    check('home velocity', model.key_qvel[home], np.zeros(model.nv))
    if verbose:
        print('[ORDER] observation joints:', OBS_ORDER)
        print('[ORDER] position observation:', POSITION_ORDER)
        print('[ORDER] action / target:', ACTION_ORDER)
        print('[ORDER] MuJoCo state:', order['state_names'], 'ctrl:', order['control_names'])
        for i, n in enumerate(ACTION_ORDER):
            print(f'[MAP] {n}: obs_joint={OBS_ORDER.index(n)}, action={i}, qpos={order["q_action"][i]}, '
                  f'qvel={order["v_action"][i]}, ctrl={a[i]}')
        print('[LIMITATION] XML Arm_Y3/Arm_P2 使用修正后的正惯量；训练 USD 中对应负惯量的 PhysX 运行时值尚未验证。')
        print('[LIMITATION] 监测实际速度上限（腿/轮2*pi，臂2rad/s），不裁剪qvel；MuJoCo接触不等价于PhysX。')


class Simulation:
    def __init__(self, model, order):
        self.model, self.order = model, order
        self.data = mujoco.MjData(model)
        self.reset()

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key('home').id)
        self.last_action = np.zeros(12, dtype=np.float32)  # ACTION_ORDER
        self.effort = np.zeros(12)  # ACTION_ORDER, cached final substep effort
        self.history = deque(maxlen=3)
        self.pd_error = 0.
        self.interval_speed = np.zeros(12)
        self.interval_effort = np.zeros(12)
        mujoco.mj_forward(self.model, self.data)

    def observation(self, command):
        d, o = self.data, self.order
        r = d.xmat[self.model.body('base_link').id].reshape(3, 3)
        relative_q = d.qpos[o['q_obs']] - np.array([DEFAULT_Q[n] for n in OBS_ORDER])
        return vector(np.concatenate([
            np.clip(d.sensor('base_com_linvel').data, -5, 5),
            np.clip(d.sensor('imu_gyro').data, -10, 10)*.25,
            r.T @ [0., 0., -1.], vector(command, 3, 'command'),
            relative_q[o['position_indices']], np.clip(d.qvel[o['v_obs']], -30, 30)*.1,
            np.clip(self.effort[o['action_to_obs']], -100, 100),
            np.clip(self.last_action, -5, 5),
        ]), OBS_DIM, 'observation')

    def expected_effort(self):
        d, o = self.data, self.order
        q, v, target = d.qpos[o['q_action']], d.qvel[o['v_action']], d.ctrl[o['actuator']]
        effort = KP*(target-q)-KD*v
        effort[4:6] = KD[4:6]*(target[4:6]-v[4:6])
        return np.clip(effort, -EFFORT_LIMIT, EFFORT_LIMIT)

    def contact_forces(self):
        net = np.zeros((self.model.nbody, 3))
        wrench = np.empty(6)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            mujoco.mj_contactForce(self.model, self.data, i, wrench)
            force = c.frame.reshape(3, 3).T @ wrench[:3]
            net[self.model.geom_bodyid[c.geom1]] -= force
            net[self.model.geom_bodyid[c.geom2]] += force
        return net

    def step(self, action):
        self.last_action = vector(action, 12, 'action').copy()
        self.data.ctrl[self.order['actuator']] = action_to_targets(action)
        self.pd_error = 0.
        self.interval_speed[:] = 0
        self.interval_effort[:] = 0
        for _ in range(DECIMATION):
            expected = self.expected_effort()
            before = self.data.time
            mujoco.mj_step(self.model, self.data)
            self.effort = self.data.actuator_force[self.order['actuator']].copy()
            self.pd_error = max(self.pd_error, float(np.max(abs(self.effort-expected))))
            if self.pd_error > 1e-8:
                raise RuntimeError(f'PD torque mismatch: {self.pd_error}')
            if abs(self.data.time-before-SIM_DT) > 1e-8 or any(w.number for w in self.data.warning):
                raise RuntimeError('MuJoCo 数值警告/自动重置')
            if not all(np.isfinite(v).all() for v in (self.data.qpos, self.data.qvel, self.data.qacc)):
                raise RuntimeError('非有限仿真状态')
            self.interval_speed = np.maximum(self.interval_speed, abs(self.data.qvel[self.order['v_action']]))
            self.interval_effort = np.maximum(self.interval_effort, abs(self.effort))
            self.history.append(self.contact_forces())
        # Preserve the pre-integration final-substep torque cache used by Isaac ImplicitActuator.
        mujoco.mj_forward(self.model, self.data)

    def termination(self, episode_seconds):
        r = self.data.xmat[self.model.body('base_link').id].reshape(3, 3)
        tilt = math.acos(float(np.clip(r[2, 2], -1, 1)))
        reasons = [f'tilt={math.degrees(tilt):.2f}deg'] if tilt > math.pi/3 else []
        if self.history:
            forces = np.linalg.norm(np.stack(self.history), axis=-1).max(axis=0)
            for n in ['base_link'] + [n for n in ACTION_ORDER if not n.startswith('Hub')]:
                if forces[self.model.body(n).id] > 1.:
                    reasons.append(f'illegal_contact:{n}={forces[self.model.body(n).id]:.2f}N')
        if self.data.time >= episode_seconds - 1e-9:
            reasons.append('timeout')
        return reasons

    def sensor_errors(self):
        d, o = self.data, self.order
        errors = {}
        for suffix, reference in [('pos', d.qpos[o['q_action']]), ('vel', d.qvel[o['v_action']]),
                                  ('torque', d.actuator_force[o['actuator']])]:
            errors[suffix] = float(max(abs(np.array([d.sensor(n+'_'+suffix).data[0] for n in ACTION_ORDER])-reference)))
        com = d.sensor('imu_linear_velocity').data + np.cross(d.sensor('imu_gyro').data, self.model.body('base_link').ipos)
        errors['com_velocity'] = float(max(abs(com-d.sensor('base_com_linvel').data)))
        if max(errors.values()) > 1e-8:
            raise RuntimeError(f'sensor/state mismatch: {errors}')
        return errors

    def record(self, obs, command, action, episode, tick):
        # Snapshot BEFORE applying this action. Cached torque/last_action belong to the previous tick.
        return dict(event='tick', episode=episode, tick=tick, total_time=tick*CONTROL_DT,
                    episode_time=self.data.time, obs=obs.tolist(), command=command.tolist(),
                    raw_action=action.tolist(), target=action_to_targets(action).tolist(),
                    previous_action=self.last_action.tolist(),
                    q=self.data.qpos[self.order['q_obs']].tolist(), dq=self.data.qvel[self.order['v_obs']].tolist(),
                    effort_last_substep=self.effort[self.order['action_to_obs']].tolist(),
                    effort_current_state=self.data.actuator_force[self.order['actuator']][self.order['action_to_obs']].tolist(),
                    base_position=self.data.xpos[self.model.body('base_link').id].tolist(),
                    sensor_errors=self.sensor_errors(), pd_error=self.pd_error,
                    previous_interval_speed_exceeded=[n for n, v, limit in zip(ACTION_ORDER, self.interval_speed, SPEED_LIMIT) if v > limit])


def print_record(record):
    fmt = lambda v: np.array2string(np.asarray(v), precision=4, suppress_small=True)
    print(f'\n[TICK] {record["tick"]} total={record["total_time"]:.3f}s episode={record["episode"]} '
          f't={record["episode_time"]:.3f}s base={fmt(record["base_position"])}')
    for name, (start, end) in OBS_SLICES.items():
        print(f'  obs[{start:02}:{end:02}] {name:20} {fmt(record["obs"][start:end])}')
    for name in ('raw_action', 'target', 'q', 'dq', 'effort_last_substep', 'effort_current_state'):
        print(f'  {name:23} {fmt(record[name])}')
    print(f'  PD error={record["pd_error"]:.3e}; sensor errors={record["sensor_errors"]}')
    print('  previous-substep speed violations:', record['previous_interval_speed_exceeded'])


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--model', type=Path, default=MODEL_XML)
    p.add_argument('--policy', type=Path, default=POLICY_PATH, help='RSL-RL training checkpoint model.pt')
    p.add_argument('--params-dir', type=Path, help='默认使用 checkpoint 真实路径旁的 params 目录')
    p.add_argument('--isaac-metadata', type=Path, help='默认查找同次训练 play_joint_torque/*/metadata.txt')
    p.add_argument('--mode', choices=['policy', 'zero'], default='policy')
    p.add_argument('--viewer', action=argparse.BooleanOptionalAction, default=True)
    p.add_argument('--realtime', action=argparse.BooleanOptionalAction, default=True)
    p.add_argument('--duration', type=float, default=40.)
    p.add_argument('--episode-seconds', type=float, default=40.)
    p.add_argument('--cmd-vx', type=float, default=0.)
    p.add_argument('--cmd-vy', type=float, default=0.)
    p.add_argument('--cmd-wz', type=float, default=0.)
    p.add_argument('--random-commands', action='store_true', help='每6-10秒重采样 vx,wz∈[-.5,.5]，PLAY无站立概率')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--auto-reset', action='store_true')
    p.add_argument('--print-interval', type=float, default=1., help='0只打印首帧；单位为仿真秒')
    p.add_argument('--log', type=Path, help='JSONL，每个策略周期保存输入、输出和状态')
    p.add_argument('--check-only', action='store_true')
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    if not np.isfinite([args.duration, args.episode_seconds, args.print_interval]).all() or min(args.duration, args.episode_seconds) <= 0 or args.print_interval < 0:
        raise ValueError('duration/episode-seconds 必须为正，print-interval 非负且有限')
    command = vector([args.cmd_vx, args.cmd_vy, args.cmd_wz], 3, 'command')
    if abs(command[0]) > .5 or command[1] != 0 or abs(command[2]) > .5:
        raise ValueError('命令范围 vx,wz∈[-.5,.5]，vy=0')
    np.set_printoptions(precision=5, suppress=True)
    audit_sources()
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    order = build_joint_order(model)
    print(f'[MODEL] {args.model.resolve()} MuJoCo={mujoco.__version__}')
    check_alignment(model, order)
    policy = Policy(args.policy, args.params_dir) if args.mode == 'policy' else None
    if policy or args.isaac_metadata:
        check_metadata(args.policy, args.isaac_metadata)
    else:
        print('[MODE] zero：无策略，仅使用已记录的 v1 关节顺序做开环诊断。')
    sources = [args.model, TASK_SOURCE, ASSET_SOURCE]
    if policy:
        sources += [policy.path, policy.params_dir/'agent.yaml', policy.params_dir/'env.yaml']
    hashes = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    for p, digest in hashes.items():
        print(f'[SOURCE] {p} sha256={digest[:16]}')
    sim = Simulation(model, order)
    reset = threading.Event()
    rng = np.random.default_rng(args.seed)
    tick = episode = 0
    next_print = next_command = 0.
    max_speed, max_effort = np.zeros(12), np.zeros(12)
    max_pd_error = 0.
    last_reasons = []
    wall_start = time.perf_counter()
    with ExitStack() as stack:
        log = None
        if args.log:
            args.log.parent.mkdir(parents=True, exist_ok=True)
            log = stack.enter_context(args.log.open('w', encoding='utf-8'))
            log.write(json.dumps(dict(event='metadata', hashes=hashes, observation_order=OBS_ORDER,
                                      position_order=POSITION_ORDER, action_order=ACTION_ORDER,
                                      obs_slices=OBS_SLICES, dt=SIM_DT, decimation=DECIMATION), ensure_ascii=False)+'\n')
        viewer = None
        if args.viewer and not args.check_only:
            from mujoco import viewer as mj_viewer
            viewer = stack.enter_context(mj_viewer.launch_passive(model, sim.data,
                key_callback=lambda key: reset.set() if key in (ord('r'), ord('R')) else None))
        while tick < math.ceil(args.duration/CONTROL_DT) and (viewer is None or viewer.is_running()):
            if reset.is_set():
                reset.clear()
                sim.reset()
                episode += 1
                next_command = 0.
            if args.random_commands and sim.data.time >= next_command:
                command[:] = [rng.uniform(-.5, .5), 0, rng.uniform(-.5, .5)]
                next_command = sim.data.time + rng.uniform(6, 10)
            obs = sim.observation(command)
            action = policy(obs) if policy else np.zeros(12, dtype=np.float32)
            record = sim.record(obs, command, action, episode, tick)
            if tick == 0 or (args.print_interval > 0 and tick*CONTROL_DT >= next_print):
                print_record(record)
                next_print = tick*CONTROL_DT + args.print_interval
            if log:
                log.write(json.dumps(record, allow_nan=False)+'\n')
            if args.check_only:
                print('[CHECK] 配置、地址、58D观测、12D动作检查通过；未推进物理仿真。')
                break
            sim.step(action)
            tick += 1
            max_speed = np.maximum(max_speed, sim.interval_speed)
            max_effort = np.maximum(max_effort, sim.interval_effort)
            max_pd_error = max(max_pd_error, sim.pd_error)
            last_reasons = sim.termination(args.episode_seconds)
            if viewer:
                viewer.sync()
            if last_reasons:
                event = dict(event='termination', episode=episode, total_time=tick*CONTROL_DT,
                             episode_time=sim.data.time, reasons=last_reasons)
                print('[DONE]', json.dumps(event, ensure_ascii=False))
                if log:
                    log.write(json.dumps(event)+'\n')
                if not args.auto_reset:
                    break
                reset.set()
            if args.realtime:
                remaining = wall_start + tick*CONTROL_DT-time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
        result = dict(event='result', policy_ticks=tick, physics_steps=tick*DECIMATION,
                      total_time=tick*CONTROL_DT, episodes_started=episode+1, pd_max_error=max_pd_error,
                      peak_substep_speed=max_speed.tolist(), peak_substep_effort=max_effort.tolist(),
                      speed_limit_exceeded=[n for n, v, limit in zip(ACTION_ORDER, max_speed, SPEED_LIMIT) if v > limit],
                      last_termination=last_reasons)
        print('[RESULT]', json.dumps(result, ensure_ascii=False))
        if log:
            log.write(json.dumps(result)+'\n')


if __name__ == '__main__':
    main()
