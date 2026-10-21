#!/usr/bin/env python3
"""Wheeled-Leg-Play-v1 的最小 MuJoCo 迁移程序（34D observation / 6D action）。

配置来源：v2_wheeled_leg_v1.py、asset/wheeled_leg_v2.py、asset/mjcf/wheeled_leg_v2.xml。
默认使用 nominal PLAY：无噪声、无质量/摩擦随机化，固定零速度命令。
策略必须是包含训练 normalizer 的 Isaac Lab 导出模型，不能传训练 checkpoint。

运行：python wheeled_leg_minimal.py --no-viewer --duration 10
检查：python wheeled_leg_minimal.py --check-only
开环：python wheeled_leg_minimal.py --mode zero --no-viewer --duration 2
按 R 重置 viewer；--log /tmp/leg.jsonl 保存每个策略周期的输入和输出。
"""
from __future__ import annotations

import argparse
from collections import deque
from contextlib import ExitStack
import hashlib
import json
import math
from pathlib import Path
import threading
import time

import mujoco
import numpy as np

WORKSPACE = Path(__file__).resolve().parents[3]
MODULE = WORKSPACE / 'modules/wheeled_mujoco'
TRAIN = WORKSPACE / 'modules/wheeled_train/wheeled_leg'
TASK = TRAIN / 'source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg'
MODEL_XML = MODULE / 'asset/mjcf/wheeled_leg_v2.xml'
POLICY_PATH = TRAIN / 'logs/rsl_rl/wheeled_leg_v1/best/exported/policy.pt'
SIM_DT, DECIMATION = 0.002, 4
CONTROL_DT = SIM_DT * DECIMATION
OBS_DIM, ACTION_DIM = 34, 6
# 来自 Wheeled-Leg-Play-v1 的 play_joint_torque/metadata.txt 中 robot.joint_names。
ISAAC_OBS_JOINT_NAME = ['HipL', 'HipR', 'KneeL', 'KneeR', 'HubL', 'HubR']
POLICY_ACTION_ORDER = ISAAC_OBS_JOINT_NAME.copy()
MUJOCO_OBS_JOINT_NAME = ['HipL', 'KneeL', 'HubL', 'HipR', 'KneeR', 'HubR']
MUJOCO_CTRL_JOINT_NAME = POLICY_ACTION_ORDER.copy()
DEFAULT_Q_BY_NAME = dict(zip(ISAAC_OBS_JOINT_NAME, [.65, -.65, 1.3, -1.3, 0., 0.]))
ACTION_SCALE_BY_SUFFIX = {'Hip': .25, 'Knee': .25, 'Hub': 4.}
KP_POLICY = dict(zip(ISAAC_OBS_JOINT_NAME, [20., 20., 30., 30., 0., 0.]))
KD_POLICY = dict.fromkeys(ISAAC_OBS_JOINT_NAME, 1.5)
TORQUE_LIMIT = dict.fromkeys(ISAAC_OBS_JOINT_NAME, 150.)
VELOCITY_LIMIT = dict.fromkeys(ISAAC_OBS_JOINT_NAME, 2 * math.pi)
TARGET_LIMIT = np.array([[.3, 1.], [-1., -.3], [.9, 1.7], [-1.7, -.9], [-6., 6.], [-6., 6.]])
OBS_SLICES = {'base_lin_vel': (0, 3), 'base_ang_vel': (3, 6), 'projected_gravity': (6, 9),
              'velocity_commands': (9, 12), 'joint_pos': (12, 16), 'joint_vel': (16, 22),
              'joint_effort': (22, 28), 'actions': (28, 34)}
BODY_MASS = dict(zip(['base_link'] + ISAAC_OBS_JOINT_NAME,
                     [10.337, 3.721, 3.721, .7847, .7847, 1.1563, 1.1563]))


def action_scale_for_joint(joint_name: str) -> float:
    for prefix, scale in ACTION_SCALE_BY_SUFFIX.items():
        if joint_name.startswith(prefix):
            return scale
    raise KeyError(f'未找到关节 {joint_name} 的缩放因子')


def indics_for_order(source_order: list[str], target_order: list[str]) -> np.ndarray:
    """返回重排索引；拒绝重复名称或缺失关节，也支持提取子集。"""
    if len(set(source_order)) != len(source_order) or len(set(target_order)) != len(target_order):
        raise ValueError('关节顺序不能有重复名称')
    if not set(target_order) <= set(source_order):
        raise ValueError(f'关节名称不匹配: {source_order} -> {target_order}')
    return np.array([source_order.index(n) for n in target_order], dtype=np.int64)


def build_joint_order(model: mujoco.MjModel) -> dict:
    """从已编译模型提取地址，不依赖 qpos[7:] 或 ctrl 的固定排列。"""
    state_names = [model.joint(i).name for i in range(model.njnt)
                   if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
    ctrl_names = [model.joint(int(model.actuator_trnid[i, 0])).name for i in range(model.nu)]
    if set(state_names) != set(ISAAC_OBS_JOINT_NAME) or len(state_names) != 6:
        raise ValueError(f'不是预期的六关节资产: {state_names}')
    if set(ctrl_names) != set(POLICY_ACTION_ORDER) or len(ctrl_names) != 6:
        raise ValueError(f'执行器必须一一绑定六关节: {ctrl_names}')
    jids = np.array([model.joint(n).id for n in ISAAC_OBS_JOINT_NAME])
    return {
        'state_names': state_names, 'ctrl_names': ctrl_names, 'joint_ids': jids,
        'qpos_ids': model.jnt_qposadr[jids].copy(), 'qvel_ids': model.jnt_dofadr[jids].copy(),
        'actuator_ids': indics_for_order(ctrl_names, POLICY_ACTION_ORDER),
        'mujoco_to_policy_obs': indics_for_order(state_names, ISAAC_OBS_JOINT_NAME),
        'mujoco_to_policy_ctrl': indics_for_order(POLICY_ACTION_ORDER, ctrl_names),
        'default_q': np.array([DEFAULT_Q_BY_NAME[n] for n in ISAAC_OBS_JOINT_NAME]),
        'action_scale_policy': np.array([action_scale_for_joint(n) for n in POLICY_ACTION_ORDER]),
        'kp_policy': np.array([KP_POLICY[n] for n in POLICY_ACTION_ORDER]),
        'kd_policy': np.array([KD_POLICY[n] for n in POLICY_ACTION_ORDER]),
        'torque_limit': np.array([TORQUE_LIMIT[n] for n in POLICY_ACTION_ORDER]),
        'velocity_limit': np.array([VELOCITY_LIMIT[n] for n in ISAAC_OBS_JOINT_NAME]),
    }


def checked_vector(value, size: int, label: str) -> np.ndarray:
    value = np.asarray(value, dtype=np.float32)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError(f'{label} 必须是有限的 ({size},) 向量，实际 {value.shape}: {value}')
    return value


def action_to_targets(raw_action: np.ndarray, order: dict) -> np.ndarray:
    # 当前 RSL-RL clip_actions=None。不要擅自把 raw action 限制到 [-1, 1]。
    action = checked_vector(raw_action, ACTION_DIM, 'action')
    offset = order['default_q'].copy()
    offset[4:] = 0.  # JointVelocityAction 使用默认速度偏置，当前为零。
    return np.clip(action * order['action_scale_policy'] + offset,
                   TARGET_LIMIT[:, 0], TARGET_LIMIT[:, 1])


def expected_effort(model, data, order) -> np.ndarray:
    targets = data.ctrl[order['actuator_ids']]
    q, dq = data.qpos[order['qpos_ids']], data.qvel[order['qvel_ids']]
    torque = order['kp_policy'] * (targets - q) - order['kd_policy'] * dq
    torque[4:] = order['kd_policy'][4:] * (targets[4:] - dq[4:])
    return np.clip(torque, -order['torque_limit'], order['torque_limit'])


def build_observation(model, data, order, command, last_action, applied_effort) -> np.ndarray:
    """PLAY 无噪声；clip 在 scale 之前。力矩为最后一个物理子步前的缓存。"""
    rotation = data.xmat[model.body('base_link').id].reshape(3, 3)
    gravity_b = rotation.T @ np.array([0., 0., -1.])
    q = data.qpos[order['qpos_ids']]
    dq = data.qvel[order['qvel_ids']]
    obs = np.concatenate([
        np.clip(data.sensor('base_com_linvel').data, -5., 5.),
        np.clip(data.sensor('imu_gyro').data, -10., 10.) * .25,
        gravity_b, checked_vector(command, 3, 'command'),
        (q - order['default_q'])[:4],
        np.clip(dq, -30., 30.) * .1,
        np.clip(checked_vector(applied_effort, 6, 'effort'), -100., 100.),
        np.clip(checked_vector(last_action, 6, 'last_action'), -5., 5.),
    ])
    return checked_vector(obs, OBS_DIM, 'observation')


class Policy:
    """CPU TorchScript / ONNX 推理；normalizer 由导出模型负责。"""
    def __init__(self, path: Path):
        if not path.is_file():
            raise FileNotFoundError(f'策略不存在: {path}；开环检查请显式使用 --mode zero')
        self.path = path
        self.kind = path.suffix.lower()
        if self.kind == '.pt':
            import torch
            self.torch = torch
            self.model = torch.jit.load(str(path), map_location='cpu').eval()
            normalizer = getattr(self.model, 'normalizer', None)
            if normalizer is None or getattr(normalizer, 'original_name', '') == 'Identity':
                raise ValueError('当前任务启用 empirical_normalization；请导出包含 normalizer 的 policy.pt')
            self.normalization = f'内置 {getattr(normalizer, "original_name", type(normalizer).__name__)}'
        elif self.kind == '.onnx':
            import onnxruntime as ort
            options = ort.SessionOptions()
            options.intra_op_num_threads = 1
            self.model = ort.InferenceSession(str(path), sess_options=options, providers=['CPUExecutionProvider'])
            inputs = self.model.get_inputs()
            if len(inputs) != 1 or inputs[0].shape[-1] != OBS_DIM or inputs[0].type != 'tensor(float)':
                raise ValueError('需要单输入 float32 [batch,34] 的非循环 ONNX policy')
            self.input_name = inputs[0].name
            self.normalization = '按 Isaac Lab 导出约定内置；ONNX shape 检查不能证明归一化统计量来源'
        else:
            raise ValueError('策略文件必须是导出的 .pt 或 .onnx')
        self(np.zeros(OBS_DIM, dtype=np.float32))  # 启动时验证 34 -> 6 及有限值。
        self.reset()

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        obs = checked_vector(obs, OBS_DIM, 'policy input')[None, :]
        if self.kind == '.pt':
            with self.torch.inference_mode():
                output = self.model(self.torch.from_numpy(obs)).cpu().numpy()
        else:
            output = self.model.run(None, {self.input_name: obs})[0]
        if output.shape != (1, ACTION_DIM):
            raise ValueError(f'policy output 应为 (1,6)，实际 {output.shape}')
        return checked_vector(output[0], ACTION_DIM, 'policy output')

    def reset(self):
        if self.kind == '.pt' and hasattr(self.model, 'reset'):
            self.model.reset()


def check_training_sources(task_path=TASK / 'v2_wheeled_leg_v1.py',
                           asset_path=TASK / 'asset/wheeled_leg_v2.py', verbose=True):
    """静态读取指定配置的字面量，不导入/启动 Isaac Sim，不执行配置文件代码。"""
    import ast
    task = ast.parse(task_path.read_text())
    asset = ast.parse(asset_path.read_text())
    checks = 0

    def cls(tree, name):
        return next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)

    def value(container, name):
        for node in container.body:
            if isinstance(node, ast.AnnAssign) and ast.unparse(node.target) == name:
                return node.value
            if isinstance(node, ast.Assign) and any(ast.unparse(t) == name for t in node.targets):
                return node.value
        raise ValueError(f'训练配置中找不到 {name}')

    def kw(call, name):
        return next(k.value for k in call.keywords if k.arg == name)

    def literal(node):
        return ast.literal_eval(node)

    def equal(label, actual, expected):
        nonlocal checks
        if actual != expected:
            raise ValueError(f'训练源码与迁移配置不一致: {label}: source={actual}, runner={expected}')
        checks += 1

    def keyword(label, call, key, expected):
        equal(label, literal(kw(call, key)), expected)

    asset_cfg = value(asset, 'WHEELED_CFG')
    actuators = kw(asset_cfg, 'actuators')
    for key, call in zip(actuators.keys, actuators.values):
        name = literal(key)
        gains = {'hip_act': (20., 1.5), 'knee_act': (30., 1.5), 'hub_act': (0., 1.5)}[name]
        for field, expected in zip(('stiffness', 'damping', 'effort_limit'), (*gains, 150.)):
            keyword(f'{name}.{field}', call, field, expected)
    keyword('soft joint limit factor', asset_cfg, 'soft_joint_pos_limit_factor', .9)
    spawn = kw(asset_cfg, 'spawn')
    keyword('self collisions', kw(spawn, 'articulation_props'), 'enabled_self_collisions', False)
    for field, expected in [('disable_gravity', False), ('linear_damping', 0.), ('angular_damping', 0.)]:
        keyword(field, kw(spawn, 'rigid_props'), field, expected)
    robot = value(cls(task, 'WheeledLegV1SceneCfg'), 'robot')
    init = kw(robot, 'init_state')
    keyword('task base position', init, 'pos', (0., 0., .58))
    keyword('task joint pose', init, 'joint_pos',
            {'HipL': .65, 'HipR': -.65, 'KneeL': 1.3, 'KneeR': -1.3, 'Hub.*': 0.})
    # 明确验证任务使用 velocity_limit_sim，而不是被当前隐式执行器忽略的旧字段。
    velocity_override = kw(robot, 'actuators').value
    keyword('legacy velocity limit', velocity_override, 'velocity_limit', None)
    equal('velocity_limit_sim', ast.dump(kw(velocity_override, 'velocity_limit_sim')),
          ast.dump(ast.parse('2.0 * math.pi', mode='eval').body))
    terrain = value(cls(task, 'WheeledLegV1SceneCfg'), 'terrain')
    for field in ('static_friction', 'dynamic_friction'):
        keyword('terrain.' + field, kw(terrain, 'physics_material'), field, 1.0)
    action_cfg = cls(task, 'ActionsCfg')
    equal('action term order', [n.targets[0].id for n in action_cfg.body if isinstance(n, ast.Assign)],
          ['joint_pos', 'joint_vel'])
    for name, patterns, scale, limits in [
        ('joint_pos', ['Hip.*', 'Knee.*'], .25,
         {'HipL': (.3, 1.), 'HipR': (-1., -.3), 'KneeL': (.9, 1.7), 'KneeR': (-1.7, -.9)}),
        ('joint_vel', ['Hub.*'], 4., {'Hub.*': (-6., 6.)}),
    ]:
        call = value(action_cfg, name)
        for field, expected in [('joint_names', patterns), ('scale', scale), ('clip', limits), ('use_default_offset', True)]:
            keyword(f'{name}.{field}', call, field, expected)
    policy_cfg = cls(cls(task, 'ObservationsCfg'), 'PolicyCfg')
    terms = [n.targets[0].id for n in policy_cfg.body if isinstance(n, ast.Assign)]
    equal('observation term order', terms, list(OBS_SLICES))
    expected_obs = {'base_lin_vel': ('base_lin_vel', (-5., 5.), None),
                    'base_ang_vel': ('base_ang_vel', (-10., 10.), .25),
                    'projected_gravity': ('projected_gravity', None, None),
                    'velocity_commands': ('generated_commands', None, None),
                    'joint_pos': ('joint_pos_rel', None, None),
                    'joint_vel': ('joint_vel_rel', (-30., 30.), .1),
                    'joint_effort': ('joint_effort', (-100., 100.), None),
                    'actions': ('last_action', (-5., 5.), None)}
    for name, (func, clip, scale) in expected_obs.items():
        call = value(policy_cfg, name)
        kwargs = {k.arg: k.value for k in call.keywords}
        equal(f'{name}.func', ast.unparse(kwargs['func']), 'mdp.' + func)
        equal(f'{name}.clip', literal(kwargs['clip']) if 'clip' in kwargs else None, clip)
        equal(f'{name}.scale', literal(kwargs['scale']) if 'scale' in kwargs else None, scale)
    pos_params = kw(value(policy_cfg, 'joint_pos'), 'params')
    entity = next(v for k, v in zip(pos_params.keys, pos_params.values) if literal(k) == 'asset_cfg')
    keyword('observation joint position selection', entity, 'joint_names', ['Hip.*', 'Knee.*'])
    init_env = next(n for n in cls(task, 'WheeledLegV1EnvCfg').body
                    if isinstance(n, ast.FunctionDef) and n.name == '__post_init__')
    equal('sim.dt', literal(value(init_env, 'self.sim.dt')), SIM_DT)
    equal('decimation', literal(value(init_env, 'self.decimation')), DECIMATION)
    play = next(n for n in cls(task, 'WheeledLegV1EnvCfg_PLAY').body
                if isinstance(n, ast.FunctionDef) and n.name == '__post_init__')
    equal('PLAY noise', literal(value(play, 'self.observations.policy.enable_corruption')), False)
    for name in ('physics_material', 'add_base_mass', 'push_robot'):
        equal(f'PLAY {name}', literal(value(play, f'self.events.{name}')), None)
    if verbose:
        print(f'[PASS] {checks} 项训练源码关键参数：资产增益、任务覆盖、动作、观测顺序/缩放、dt/decimation、PLAY随机化。')
        print('[SOURCE] 这是当前源码静态核对；checkpoint 的训练配置和运行时 USD 仍以该次训练记录为准。')


def check_alignment(model, order, verbose=True) -> None:
    """失败就停止，避免打印 PASS 后继续使用不匹配的模型。"""
    errors = []

    def check(label, actual, expected, atol=1e-7):
        ok = np.shape(actual) == np.shape(expected) and np.allclose(actual, expected, atol=atol, rtol=1e-6)
        if verbose:
            print(f'[{"PASS" if ok else "FAIL"}] {label}: actual={actual} expected={expected}')
        if not ok:
            errors.append(label)

    check('nq/nv/nu', [model.nq, model.nv, model.nu], [13, 12, 6])
    check('物理步长 / 控制周期', [model.opt.timestep, model.opt.timestep * DECIMATION], [SIM_DT, CONTROL_DT])
    check('重力', model.opt.gravity, [0, 0, -9.81])
    ids, aids = order['joint_ids'], order['actuator_ids']
    check('传动类型 JOINT', model.actuator_trntype[aids], np.zeros(6))
    check('gear', model.actuator_gear[aids], np.tile([1., 0, 0, 0, 0, 0], (6, 1)))
    check('ctrl limited', model.actuator_ctrllimited[aids], np.ones(6))
    check('force limited', model.actuator_forcelimited[aids], np.ones(6))
    check('joint force limited', model.jnt_actfrclimited[ids], np.ones(6))
    check('ctrlrange', model.actuator_ctrlrange[aids], TARGET_LIMIT)
    check('forcerange', model.actuator_forcerange[aids], np.tile([-150., 150.], (6, 1)))
    check('joint actuatorfrcrange', model.jnt_actfrcrange[ids], np.tile([-150., 150.], (6, 1)))
    check('gain type FIXED', model.actuator_gaintype[aids], np.zeros(6))
    check('bias type AFFINE', model.actuator_biastype[aids], np.ones(6))
    check('activation dynamics NONE', model.actuator_dyntype[aids], np.zeros(6))
    check('gain', model.actuator_gainprm[aids, 0], [20, 20, 30, 30, 1.5, 1.5])
    check('position bias', model.actuator_biasprm[aids, 1], -order['kp_policy'])
    check('velocity bias', model.actuator_biasprm[aids, 2], -order['kd_policy'])
    check('constant bias', model.actuator_biasprm[aids, 0], np.zeros(6))
    for field in ('dof_damping', 'dof_armature', 'dof_frictionloss'):
        check(field, getattr(model, field), np.zeros(model.nv))
    check('joint limits enabled', model.jnt_limited[ids], [1, 1, 1, 1, 0, 0])
    check('leg hard limits', model.jnt_range[ids[:4]], np.tile([-3.14, 3.14], (4, 1)))
    for name, mass in BODY_MASS.items():
        check(f'{name} mass', float(model.body(name).mass[0]), mass)
    base = model.body('base_link').id
    check('COM site', model.site('base_com').pos, model.body_ipos[base])
    check('COM site body', int(model.site('base_com').bodyid[0]), base)
    for site in ('imu', 'base_com'):
        check(f'{site} orientation', model.site(site).quat, [1, 0, 0, 0])
    required = {'base_com_linvel': (mujoco.mjtSensor.mjSENS_VELOCIMETER, 'base_com'),
                'imu_gyro': (mujoco.mjtSensor.mjSENS_GYRO, 'imu')}
    for name, (kind, site) in required.items():
        s = model.sensor(name)
        check(f'{name} type/dim/site', [int(s.type[0]), int(s.dim[0]), int(s.objid[0])],
              [int(kind), 3, model.site(site).id])
    for i, name in enumerate(ISAAC_OBS_JOINT_NAME):
        for suffix, kind, object_id in [('pos', mujoco.mjtSensor.mjSENS_JOINTPOS, ids[i]),
                                        ('vel', mujoco.mjtSensor.mjSENS_JOINTVEL, ids[i]),
                                        ('torque', mujoco.mjtSensor.mjSENS_ACTUATORFRC, aids[i])]:
            s = model.sensor(name + '_' + suffix)
            check(f'{name}_{suffix} type/dim/object', [int(s.type[0]), int(s.dim[0]), int(s.objid[0])],
                  [int(kind), 1, int(object_id)])
    for g in range(model.ngeom):
        if model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0:
            continue
        ground = model.geom_bodyid[g] == 0
        check(f'geom[{g}] collision mask/condim', [model.geom_contype[g], model.geom_conaffinity[g], model.geom_condim[g]],
              [1, 2, 3] if ground else [2, 1, 3])
        check(f'geom[{g}] friction', model.geom_friction[g], [1., 0., 0.])
    home = model.key('home').id
    check('home base', model.key_qpos[home, :7], [0, 0, .58, 1, 0, 0, 0])
    check('home joints', model.key_qpos[home, order['qpos_ids']], order['default_q'])
    check('home controls', model.key_ctrl[home, aids], action_to_targets(np.zeros(6), order))
    if errors:
        raise ValueError('模型未对齐: ' + ', '.join(errors))
    if verbose:
        print('[ORDER] Isaac obs/action:', ISAAC_OBS_JOINT_NAME)
        print('[ORDER] MuJoCo state:', order['state_names'], 'control:', order['ctrl_names'])
        for i, name in enumerate(ISAAC_OBS_JOINT_NAME):
            print(f'[MAP] {name}: obs/action={i}, qpos={order["qpos_ids"][i]}, '
                  f'qvel={order["qvel_ids"][i]}, actuator/ctrl={aids[i]}')
        print('[LIMITATION] 实际速度上限 2*pi 未由 MuJoCo 硬约束；只监测超限，不直接裁剪 qvel。')
        print('[LIMITATION] PhysX/MuJoCo 接触求解不同；按要求使用原 base_link.STL。')
        print('[LIMITATION] 本地参数和绑定检查不等于 Isaac 轨迹一致性验证；策略版本/normalizer 仍需对应 checkpoint。')


class Simulation:
    def __init__(self, model, order):
        self.model, self.order = model, order
        self.data = mujoco.MjData(model)
        self.reset()

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key('home').id)
        self.last_action = np.zeros(6, dtype=np.float32)
        self.applied_effort = np.zeros(6)  # Isaac reset 清零执行器力矩缓存。
        self.contact_history = deque(maxlen=3)
        self.last_pd_error = 0.
        self.peak_speed = self.peak_torque = 0.
        self.steps = 0
        mujoco.mj_forward(self.model, self.data)

    def observation(self, command):
        return build_observation(self.model, self.data, self.order, command, self.last_action, self.applied_effort)

    def contact_forces(self):
        # 每个刚体累加世界系接触力，近似 ContactSensor.net_forces_w。
        forces = np.zeros((self.model.nbody, 3))
        force = np.empty(6)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            mujoco.mj_contactForce(self.model, self.data, i, force)
            world_force = c.frame.reshape(3, 3).T @ force[:3]
            b1, b2 = self.model.geom_bodyid[c.geom1], self.model.geom_bodyid[c.geom2]
            forces[b1] -= world_force
            forces[b2] += world_force
        return forces

    def step(self, action):
        action = checked_vector(action, 6, 'action')
        self.data.ctrl[self.order['actuator_ids']] = action_to_targets(action, self.order)
        self.last_action = action.copy()
        self.last_pd_error = 0.
        for _ in range(DECIMATION):
            # Isaac ImplicitActuator 每个子步写入目标时，以子步前 q/dq 计算 applied_torque。
            expected = expected_effort(self.model, self.data, self.order)
            before = self.data.time
            mujoco.mj_step(self.model, self.data)
            self.applied_effort = self.data.actuator_force[self.order['actuator_ids']].copy()
            error = float(np.max(np.abs(expected - self.applied_effort)))
            self.last_pd_error = max(self.last_pd_error, error)
            if error > 1e-8:
                raise RuntimeError(f'PD law 与执行器力矩不一致: max_error={error}')
            if abs(self.data.time - before - SIM_DT) > 1e-8 or any(w.number for w in self.data.warning):
                raise RuntimeError('MuJoCo 数值警告或自动重置；停止 rollout')
            if not all(np.isfinite(x).all() for x in (self.data.qpos, self.data.qvel, self.data.qacc)):
                raise RuntimeError('仿真出现非有限状态')
            self.peak_speed = max(self.peak_speed, float(np.max(np.abs(self.data.qvel[self.order['qvel_ids']]))))
            self.peak_torque = max(self.peak_torque, float(np.max(np.abs(self.applied_effort))))
            self.contact_history.append(self.contact_forces())
            self.steps += 1
        # mj_step 后的传感器/位姿通常仍对应积分前状态；刷新运动学，保留上面的力矩缓存。
        mujoco.mj_forward(self.model, self.data)

    def termination(self, episode_seconds=40.):
        reasons = []
        r = self.data.xmat[self.model.body('base_link').id].reshape(3, 3)
        tilt = math.acos(float(np.clip(r[2, 2], -1., 1.)))
        if tilt > math.pi / 3:
            reasons.append(f'tilt={math.degrees(tilt):.1f}deg > 60deg')
        if self.contact_history:
            history = np.stack(self.contact_history)
            for name in ['base_link', 'HipL', 'HipR', 'KneeL', 'KneeR']:
                peak = np.linalg.norm(history[:, self.model.body(name).id], axis=-1).max()
                if peak > 1.:
                    reasons.append(f'contact:{name}={peak:.2f}N > 1N')
        if self.data.time >= episode_seconds - 1e-9:
            reasons.append('timeout')
        return reasons

    def sensor_residuals(self):
        q = np.array([self.data.sensor(n + '_pos').data[0] for n in ISAAC_OBS_JOINT_NAME])
        dq = np.array([self.data.sensor(n + '_vel').data[0] for n in ISAAC_OBS_JOINT_NAME])
        tau = np.array([self.data.sensor(n + '_torque').data[0] for n in ISAAC_OBS_JOINT_NAME])
        com_expected = self.data.sensor('imu_linear_velocity').data + np.cross(
            self.data.sensor('imu_gyro').data, self.model.body('base_link').ipos)
        return {'q': float(np.max(abs(q - self.data.qpos[self.order['qpos_ids']]))),
                'dq': float(np.max(abs(dq - self.data.qvel[self.order['qvel_ids']]))),
                'torque': float(np.max(abs(tau - self.data.actuator_force[self.order['actuator_ids']]))),
                'com_velocity': float(np.max(abs(com_expected - self.data.sensor('base_com_linvel').data)))}

    def record(self, command, obs, action, episode, total_time):
        dq = self.data.qvel[self.order['qvel_ids']]
        residual = self.sensor_residuals()
        if max(residual.values()) > 1e-8:
            raise RuntimeError(f'传感器和模型状态不一致: {residual}')
        return {'episode': episode, 'total_time': total_time, 'episode_time': self.data.time,
                'sensor_max_errors': residual,
                'obs': obs.tolist(), 'command': command.tolist(), 'raw_action': action.tolist(),
                'target': action_to_targets(action, self.order).tolist(),
                'q': self.data.qpos[self.order['qpos_ids']].tolist(), 'dq': dq.tolist(),
                'previous_action': self.last_action.tolist(), 'effort_last_substep': self.applied_effort.tolist(),
                'effort_current_state': self.data.actuator_force[self.order['actuator_ids']].tolist(),
                'pd_max_error': self.last_pd_error,
                'speed_limit_exceeded': [n for n, v in zip(ISAAC_OBS_JOINT_NAME, dq) if abs(v) > 2 * math.pi],
                'base_position': self.data.xpos[self.model.body('base_link').id].tolist()}


def print_record(record):
    fmt = lambda x: np.array2string(np.asarray(x), precision=4, suppress_small=True)
    print(f'\n[TICK] total={record["total_time"]:.3f}s episode={record["episode"]} '
          f't={record["episode_time"]:.3f}s base={fmt(record["base_position"])}')
    for name, (start, end) in OBS_SLICES.items():
        print(f'  obs[{start:02d}:{end:02d}] {name:20s} {fmt(record["obs"][start:end])}')
    for name in ('raw_action', 'target', 'q', 'dq', 'effort_last_substep', 'effort_current_state'):
        print(f'  {name:22s} {fmt(record[name])}')
    print(f'  PD error={record["pd_max_error"]:.3e}; speed>2*pi={record["speed_limit_exceeded"]}')
    print('  sensor/state errors:', record['sensor_max_errors'])


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--model', type=Path, default=MODEL_XML)
    p.add_argument('--policy', type=Path, default=POLICY_PATH, help='包含 normalizer 的 .pt/.onnx 导出策略')
    p.add_argument('--mode', choices=['policy', 'zero'], default='policy', help='zero 是显式开环诊断，不是平衡控制器')
    p.add_argument('--viewer', action=argparse.BooleanOptionalAction, default=True)
    p.add_argument('--realtime', action=argparse.BooleanOptionalAction, default=True)
    p.add_argument('--duration', type=float, default=40., help='总仿真时长，秒')
    p.add_argument('--episode-seconds', type=float, default=40., help='PLAY 超时，默认40秒')
    p.add_argument('--cmd-vx', type=float, default=0.)
    p.add_argument('--cmd-vy', type=float, default=0.)
    p.add_argument('--cmd-wz', type=float, default=0.)
    p.add_argument('--random-commands', action='store_true', help='PLAY 范围内每6到10秒重采样命令')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--isaac-metadata', type=Path, help='可选 PLAY torque metadata.txt，核对实际关节顺序和step_dt')
    p.add_argument('--print-interval', type=float, default=1., help='仿真时间间隔，0表示只打印首帧')
    p.add_argument('--log', type=Path, help='JSONL：每个策略周期完整的34D输入、动作、目标和状态')
    p.add_argument('--auto-reset', action='store_true', help='终止后重置继续；默认打印原因并停止')
    p.add_argument('--check-only', action='store_true', help='检查参数、首帧观测和策略输出后退出')
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    numeric = [args.duration, args.episode_seconds, args.print_interval, args.cmd_vx, args.cmd_vy, args.cmd_wz]
    if not np.isfinite(numeric).all() or args.duration <= 0 or args.episode_seconds <= 0 or args.print_interval < 0:
        raise ValueError('时长必须为正，打印间隔非负，参数必须有限')
    command = np.array([args.cmd_vx, args.cmd_vy, args.cmd_wz], dtype=np.float32)
    if abs(command[0]) > .5 or command[1] != 0 or abs(command[2]) > .5:
        raise ValueError('当前任务命令范围: vx=[-.5,.5], vy=0, wz=[-.5,.5]')
    np.set_printoptions(precision=5, suppress=True)
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    order = build_joint_order(model)
    print(f'[MODEL] {args.model.resolve()} MuJoCo={mujoco.__version__}')
    for path in (args.model, TASK / 'v2_wheeled_leg_v1.py', TASK / 'asset/wheeled_leg_v2.py'):
        print(f'[SOURCE] {path} sha256={hashlib.sha256(path.read_bytes()).hexdigest()[:16]}')
    check_training_sources()
    if args.isaac_metadata:
        entries = dict(line.strip().split('=', 1) for line in args.isaac_metadata.read_text().splitlines() if '=' in line)
        if entries.get('joint_names', '').split(',') != ISAAC_OBS_JOINT_NAME or not math.isclose(float(entries['step_dt']), CONTROL_DT):
            raise ValueError('Isaac metadata 的 joint_names/step_dt 与当前迁移定义不一致')
        print(f'[PASS] Isaac runtime metadata: {args.isaac_metadata}')
    else:
        print('[ORDER SOURCE] 默认顺序来自已有 v1 PLAY torque 日志；可用 --isaac-metadata 核对指定训练运行。')
    check_alignment(model, order)
    policy = Policy(args.policy.resolve()) if args.mode == 'policy' else None
    if policy:
        print(f'[POLICY] {policy.path} 34D -> 6D; {policy.normalization}; 本程序不重复归一化')
    else:
        print('[MODE] zero：固定零动作，只测试接口和开环动力学')
    sim = Simulation(model, order)
    rng = np.random.default_rng(args.seed)
    reset_requested = threading.Event()
    episode, ticks = 0, 0
    next_print, next_command = 0., 0.
    tick_limit = math.ceil(args.duration / CONTROL_DT)
    wall_start = time.perf_counter()
    peak_speed, peak_torque = 0., 0.
    with ExitStack() as stack:
        log = None
        if args.log:
            args.log.parent.mkdir(parents=True, exist_ok=True)
            log = stack.enter_context(args.log.open('w', encoding='utf-8'))
        viewer = None
        if args.viewer and not args.check_only:
            from mujoco import viewer as mj_viewer
            viewer = stack.enter_context(mj_viewer.launch_passive(
                model, sim.data, key_callback=lambda key: reset_requested.set() if key in (ord('r'), ord('R')) else None))
        while ticks < tick_limit and (viewer is None or viewer.is_running()):
            if reset_requested.is_set():
                reset_requested.clear()
                sim.reset()
                if policy:
                    policy.reset()
                episode += 1
                next_command = 0.
            if args.random_commands and sim.data.time >= next_command:
                command[:] = [rng.uniform(-.5, .5), 0., rng.uniform(-.5, .5)]
                next_command = sim.data.time + rng.uniform(6., 10.)
            obs = sim.observation(command)
            action = policy(obs) if policy else np.zeros(6, dtype=np.float32)
            record = sim.record(command, obs, action, episode, ticks * CONTROL_DT)
            if ticks == 0 or (args.print_interval > 0 and ticks * CONTROL_DT >= next_print):
                print_record(record)
                next_print = ticks * CONTROL_DT + args.print_interval
            if log:
                log.write(json.dumps(record, allow_nan=False) + '\n')
            if args.check_only:
                print('[CHECK] 参数、地址映射、34D观测及6D动作检查通过；未执行物理步。')
                break
            sim.step(action)
            ticks += 1
            peak_speed = max(peak_speed, sim.peak_speed)
            peak_torque = max(peak_torque, sim.peak_torque)
            reasons = sim.termination(args.episode_seconds)
            if viewer:
                viewer.sync()
            if reasons:
                event = {'event': 'termination', 'episode': episode, 'total_time': ticks * CONTROL_DT,
                         'episode_time': sim.data.time, 'reasons': reasons}
                print('[DONE]', json.dumps(event, ensure_ascii=False))
                if log:
                    log.write(json.dumps(event) + '\n')
                if not args.auto_reset:
                    break
                reset_requested.set()
            if args.realtime:
                remaining = wall_start + ticks * CONTROL_DT - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
        print(f'[RESULT] policy_ticks={ticks}, physics_steps={ticks * DECIMATION}, '
              f'total_sim_time={ticks * CONTROL_DT:.3f}s, episodes_started={episode + 1}, '
              f'peak_substep_speed={peak_speed:.4f}rad/s, peak_substep_torque={peak_torque:.4f}Nm')


if __name__ == '__main__':
    main()
