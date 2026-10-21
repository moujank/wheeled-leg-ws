# SPDX-License-Identifier: BSD-3-Clause
"""Task-aware bilateral rewards and RSL-RL reflection augmentation.

USD joint axes were checked in wheeled_leg_v2: ALL three right-hand joint
axes oppose their left counterparts. Reflection swaps AND negates Hip/Knee/Hub
coordinates. Body vectors use polar/axial reflection rules, not joint rules.
"""
from __future__ import annotations

import math
import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import wrap_to_pi

__all__ = [
    'mirrored_leg_position_l2', 'mirrored_leg_velocity_l2', 'mirrored_leg_action_l2',
    'mirrored_leg_action_rate_l2', 'straight_wheel_speed_balance', 'wheel_slip_l2',
    'single_side_support_penalty', 'straight_yaw_drift_l2', 'compute_symmetric_states',
    'install_symmetry_normalization',
]


def _paired_name(name):
    if name in ('HipL', 'KneeL', 'HubL'):
        return name[:-1] + 'R'
    if name in ('HipR', 'KneeR', 'HubR'):
        return name[:-1] + 'L'
    if name == 'base_link':
        return name
    raise ValueError(f'No reflection rule for robot channel {name!r}')


def _indices(names):
    return [names.index(_paired_name(name)) for name in names]


def _selected(names, ids):
    if isinstance(ids, slice):
        return list(names[ids])
    return [names[int(i)] for i in ids]


def _upright(env, degrees=45.):
    return env.scene['robot'].data.projected_gravity_b[:, 2] < -math.cos(math.radians(degrees))


def _straight(env, command_name='base_velocity', yaw_threshold=.25, min_speed=0.):
    command = env.command_manager.get_command(command_name)
    return (command[:, 2].abs() <= yaw_threshold) & (command[:, 0].abs() >= min_speed)


def _wheel_ids(env):
    if not hasattr(env, '_coordination_wheel_ids'):
        env._coordination_wheel_ids = env.scene['robot'].find_bodies(['HubL', 'HubR'], preserve_order=True)[0]
    return env._coordination_wheel_ids


def _coordination_weight(env, mode):
    weight = _upright(env).float()
    if mode in ('terrain', 'obstacle'):
        wheels = env.scene['robot'].data.body_pos_w[:, _wheel_ids(env), :]
        # Unequal ground elevation requires unequal leg configuration. Retain a
        # weak cost so persistent lateral bias still matters on difficult ground.
        relief = (wheels[:, 0, 2] - wheels[:, 1, 2]).abs()
        weight *= 1. - .75 * ((relief - .015) / .085).clamp(0, 1)
        if mode == 'obstacle':
            from .stability import stair_climb_weight
            weight *= 1. - .8 * stair_climb_weight(env)
    elif mode == 'jump':
        term = env.command_manager.get_term('base_velocity')
        term.update_state()
        # Synchronize crouch/launch; allow corrective leg motion in flight.
        weight *= torch.where(term.in_flight & ~term.flight_ended, .5, 1.)
    elif mode != 'flat':
        raise ValueError(f'Unknown coordination mode: {mode}')
    return weight


def _leg_weight(env, mode, command_name):
    # Permit differential leg motion during commanded turns, including patrol
    # reversals. Standing and random emergency stops remain constrained.
    return _coordination_weight(env, mode) * _straight(env, command_name, .35)


def mirrored_leg_position_l2(env, left_asset_cfg: SceneEntityCfg, right_asset_cfg: SceneEntityCfg,
                             deadband=.03, command_name='base_velocity', mode='flat'):
    data = env.scene[left_asset_cfg.name].data
    error = wrap_to_pi(data.joint_pos[:, left_asset_cfg.joint_ids] + data.joint_pos[:, right_asset_cfg.joint_ids])
    return (error.abs() - deadband).clamp(min=0).square().sum(-1) * _leg_weight(env, mode, command_name)


def mirrored_leg_velocity_l2(env, left_asset_cfg: SceneEntityCfg, right_asset_cfg: SceneEntityCfg,
                             command_name='base_velocity', mode='flat'):
    data = env.scene[left_asset_cfg.name].data
    error = data.joint_vel[:, left_asset_cfg.joint_ids] + data.joint_vel[:, right_asset_cfg.joint_ids]
    return error.square().sum(-1) * _leg_weight(env, mode, command_name)


def _action_info(env, name):
    manager = env.action_manager
    i = manager.active_terms.index(name)
    start = sum(manager.action_term_dim[:i])
    names = list(manager.get_term(name)._joint_names)
    return slice(start, start + manager.action_term_dim[i]), _indices(names)


def mirrored_leg_action_l2(env, action_name='joint_pos', command_name='base_velocity', mode='flat'):
    sl, pair = _action_info(env, action_name)
    action = env.action_manager.action[:, sl]
    # Each physical pair occurs twice in the exchanged tensor.
    return .5 * (action + action[:, pair]).square().sum(-1) * _leg_weight(env, mode, command_name)


def mirrored_leg_action_rate_l2(env, action_name='joint_pos', command_name='base_velocity', mode='flat'):
    sl, pair = _action_info(env, action_name)
    delta = (env.action_manager.action - env.action_manager.prev_action)[:, sl]
    return .5 * (delta + delta[:, pair]).square().sum(-1) * _leg_weight(env, mode, command_name)


def straight_wheel_speed_balance(env, wheel_asset_cfg: SceneEntityCfg, command_name='base_velocity',
                                 yaw_threshold=.15, min_speed=0., mode='flat'):
    wheels = env.scene[wheel_asset_cfg.name].data.joint_vel[:, wheel_asset_cfg.joint_ids]
    # Opposing USD wheel axes: equal physical rolling speed means qdotL=-qdotR.
    return wheels.sum(-1).square() * _straight(env, command_name, yaw_threshold, min_speed) * _coordination_weight(env, mode)


def wheel_slip_l2(env, wheel_asset_cfg: SceneEntityCfg, wheel_sensor_cfg: SceneEntityCfg,
                  wheel_radius=.0935, contact_force_threshold=2., contact_grace_s=.03, mode='flat'):
    """Horizontal velocity of each supported wheel's bottom tread point.

    Uses world angular velocity (axis/sign independent) and LINK-frame origin
    velocity at the wheel centre, rather than COM velocity. This is an
    approximation to the contact point on slopes/edges; terrain gates reduce
    its cost there. Each wheel's contact mask is independent.
    """
    data = env.scene[wheel_asset_cfg.name].data
    sensor = env.scene.sensors[wheel_sensor_cfg.name].data
    ids, contact_ids = wheel_asset_cfg.body_ids, wheel_sensor_cfg.body_ids
    velocity = data.body_link_lin_vel_w[:, ids, :]
    angular = data.body_ang_vel_w[:, ids, :]
    radius = torch.zeros_like(velocity)
    radius[..., 2] = -wheel_radius
    tread = velocity + torch.linalg.cross(angular, radius, dim=-1)
    contact = (sensor.net_forces_w[:, contact_ids, 2] > contact_force_threshold)
    contact &= sensor.current_contact_time[:, contact_ids] > contact_grace_s
    slip = (tread[..., :2].square().sum(-1) * contact).sum(-1)
    return slip * _coordination_weight(env, mode)


def single_side_support_penalty(env, sensor_cfg: SceneEntityCfg, command_name='base_velocity',
                                force_threshold=5., imbalance_deadband=.15, unilateral_grace_s=.18,
                                mode='flat'):
    """Load imbalance plus sustained unilateral support, with landing grace.

    Air/contact timers are maintained and reset by ContactSensor. No separate
    reward timer can accumulate twice when the reward is queried again.
    """
    data = env.scene.sensors[sensor_cfg.name].data
    ids = sensor_cfg.body_ids
    force = data.net_forces_w[:, ids, 2].clamp(min=0.)
    contact = force > force_threshold
    both = contact.all(-1)
    one = contact.any(-1) & ~both
    imbalance = (force[:, 0] - force[:, 1]).abs() / force.sum(-1).clamp(min=force_threshold)
    # First landing wheel gets a short interval to settle before load balancing.
    settled = data.current_contact_time[:, ids].amin(-1) > .06
    load = (imbalance - imbalance_deadband).clamp(min=0.).square() * both * settled
    air_time = torch.where(~contact, data.current_air_time[:, ids], 0.).amax(-1)
    contact_time = torch.where(contact, data.current_contact_time[:, ids], 0.).amax(-1)
    duration = torch.minimum(air_time, contact_time)
    unilateral = ((duration - unilateral_grace_s) / .3).clamp(0, 1).square() * one
    return (load + unilateral) * _leg_weight(env, mode, command_name)


def straight_yaw_drift_l2(env, command_name='base_velocity', yaw_threshold=.2, min_speed=0.):
    command = env.command_manager.get_command(command_name)
    error = env.scene['robot'].data.root_ang_vel_b[:, 2] - command[:, 2]
    return error.square() * _straight(env, command_name, yaw_threshold, min_speed) * _upright(env)


def _action_map(env):
    permutation, signs, offset = [], [], 0
    for name, width in zip(env.action_manager.active_terms, env.action_manager.action_term_dim):
        term = env.action_manager.get_term(name)
        names = list(term._joint_names)
        if len(names) != width:
            raise ValueError(f'Unsupported action term: {name}')
        permutation.extend(offset + i for i in _indices(names))
        signs.extend([-1.] * width)
        offset += width
    return permutation, signs


def _observation_map(env, group):
    """Build the permutation from resolved managers; never assume term order."""
    manager = env.observation_manager
    permutation, signs = [], []
    configs = manager._group_obs_term_cfgs[group]
    terms = manager.active_terms[group]
    dimensions = manager.group_obs_term_dim[group]
    for name, dims, cfg in zip(terms, dimensions, configs):
        if len(dims) != 1:
            raise ValueError(f'Symmetry requires flat terms, got {group}/{name}: {dims}')
        width = dims[0]
        p, s = list(range(width)), [1.] * width
        if name in ('base_lin_vel', 'projected_gravity'):
            s = [1., -1., 1.]
        elif name == 'base_ang_vel':
            s = [-1., 1., -1.]
        elif name == 'velocity_commands':
            s = [1., -1., -1.]
        elif name in ('joint_pos', 'joint_vel', 'joint_effort'):
            asset_cfg = cfg.params.get('asset_cfg', SceneEntityCfg('robot'))
            robot = env.scene[asset_cfg.name]
            names = _selected(robot.joint_names, asset_cfg.joint_ids)
            p, s = _indices(names), [-1.] * width
            # Relative-state reflection is linear only for mirrored defaults.
            if name != 'joint_effort':
                default = robot.data.default_joint_pos if name == 'joint_pos' else robot.data.default_joint_vel
                values = default[:, asset_cfg.joint_ids]
                if not torch.allclose(values, -values[:, p], atol=1e-5):
                    raise ValueError(f'{name} defaults are not bilaterally symmetric')
        elif name == 'actions':
            p, s = _action_map(env)
        elif name == 'body_contact':
            sensor_cfg = cfg.params['sensor_cfg']
            sensor = env.scene.sensors[sensor_cfg.name]
            p = _indices(_selected(sensor.body_names, sensor_cfg.body_ids))
        elif name == 'terrain_heights':
            sensor_cfg = cfg.params.get('sensor_cfg', SceneEntityCfg('height_scanner'))
            pattern = env.scene.sensors[sensor_cfg.name].cfg.pattern_cfg
            starts, _ = pattern.func(pattern, env.device)
            reflected = starts.clone()
            reflected[:, 1] *= -1
            distance = torch.cdist(reflected, starts, compute_mode='donot_use_mm_for_euclid_dist')
            best, p_tensor = distance.min(-1)
            if not (best < 1e-5).all():
                raise ValueError('Ray pattern is not symmetric about Y=0')
            p = p_tensor.tolist()
        elif name == 'jump_course':
            if width != 12:
                raise ValueError('Unexpected jump observation width')
            s[1] = -1.
            p[10], p[11] = 11, 10
        else:
            raise ValueError(f'No observation reflection rule for {group}/{name}')
        if len(p) != width or len(s) != width:
            raise ValueError(f'Reflection width mismatch for {group}/{name}')
        # Unequal preprocessing scales break the physical signed permutation.
        if cfg.scale is not None and not isinstance(cfg.scale, (float, int)):
            scale = torch.as_tensor(cfg.scale)
            if scale.numel() not in (1, width):
                raise ValueError(f'Observation scale width mismatch for {name}')
            if scale.numel() > 1 and not torch.allclose(scale, scale[p]):
                raise ValueError(f'Asymmetric observation scale for {name}')
        offset = len(permutation)
        permutation.extend(offset + i for i in p)
        signs.extend(s)
    return permutation, signs


def _map(env, kind):
    env = env.unwrapped if hasattr(env, 'unwrapped') else env
    if not hasattr(env, '_bilateral_maps'):
        env._bilateral_maps = {}
    if kind not in env._bilateral_maps:
        p, s = _action_map(env) if kind == 'actions' else _observation_map(env, kind)
        p = torch.tensor(p, device=env.device, dtype=torch.long)
        s = torch.tensor(s, device=env.device)
        if not torch.equal(p[p], torch.arange(len(p), device=env.device)) or not torch.equal(s * s[p], torch.ones_like(s)):
            raise ValueError(f'Reflection is not an involution for {kind}')
        env._bilateral_maps[kind] = (p, s)
    return env._bilateral_maps[kind]


def _reflect(values, mapping):
    p, s = mapping
    if values.shape[-1] != len(p):
        raise ValueError(f'Reflection expects {len(p)} channels, got {values.shape[-1]}')
    return values[..., p] * s


@torch.no_grad()
def compute_symmetric_states(env, obs=None, actions=None, obs_type='policy'):
    """RSL-RL callback: original batch followed by one sagittal reflection."""
    base = env.unwrapped if hasattr(env, 'unwrapped') else env
    obs_aug = actions_aug = None
    if obs is not None:
        mapping = _map(base, obs_type)
        normalizer = getattr(base, '_bilateral_normalizers', {}).get(obs_type)
        if normalizer is None:
            mirrored = _reflect(obs, mapping)
        else:
            physical = normalizer.inverse(obs)
            mirrored = (_reflect(physical, mapping) - normalizer._mean) / (normalizer._std + normalizer.eps)
        obs_aug = torch.cat((obs, mirrored), dim=0)
    if actions is not None:
        actions_aug = torch.cat((actions, _reflect(actions, _map(base, 'actions'))), dim=0)
    return obs_aug, actions_aug


@torch.no_grad()
def install_symmetry_normalization(runner, env):
    """Use symmetric moments and preserve a loaded policy's physical outputs.

    Must run AFTER checkpoint load. Mirrored moments make reflection commute
    with normalization even while rollout statistics evolve. Reparameterizing
    each first linear layer preserves actor/critic outputs for every raw input.
    Normalizer buffer names and checkpoint format are unchanged.
    """
    from types import MethodType
    base = env.unwrapped if hasattr(env, 'unwrapped') else env
    symmetry = runner.alg.symmetry
    if not symmetry or not (symmetry['use_data_augmentation'] or symmetry['use_mirror_loss']):
        return
    _map(base, 'actions')
    base._bilateral_normalizers = {}
    policy = runner.alg.policy
    for group, normalizer, network in (
        ('policy', runner.obs_normalizer, policy.actor),
        ('critic', runner.privileged_obs_normalizer, policy.critic),
    ):
        mapping = _map(base, group)
        if not runner.empirical_normalization:
            continue
        if getattr(normalizer, '_bilateral_installed', False):
            raise RuntimeError('Install symmetry normalization only once, after loading')
        old_mean, old_std = normalizer._mean.clone(), normalizer._std.clone()
        p, s = mapping
        mean = .5 * (old_mean + old_mean[:, p] * s)
        var = .5 * (normalizer._var + normalizer._var[:, p]) + .25 * (old_mean - old_mean[:, p] * s).square()
        std = var.clamp(min=0).sqrt()
        layer = next((m for m in network.modules() if isinstance(m, torch.nn.Linear)), None)
        if layer is None or layer.in_features != mean.shape[-1]:
            raise ValueError(f'Unsupported {group} network for normalization migration')
        old_weight = layer.weight.clone()
        layer.weight.mul_(((std + normalizer.eps) / (old_std + normalizer.eps)).squeeze(0))
        layer.bias.add_((old_weight @ ((mean - old_mean) / (old_std + normalizer.eps)).T).squeeze(-1))
        normalizer._mean.copy_(mean)
        normalizer._var.copy_(var)
        normalizer._std.copy_(std)
        # Clear Adam moments only for reparameterized tensors. All remaining
        # optimizer state and the resume iteration counter are retained.
        if not torch.allclose(old_mean, mean) or not torch.allclose(old_std, std):
            runner.alg.optimizer.state.pop(layer.weight, None)
            runner.alg.optimizer.state.pop(layer.bias, None)
        original_update = normalizer.update

        def update_with_reflection(self, values, original=original_update, mirror=mapping):
            original(torch.cat((values, _reflect(values, mirror)), dim=0))

        normalizer.update = MethodType(update_with_reflection, normalizer)
        normalizer._bilateral_installed = True
        base._bilateral_normalizers[group] = normalizer
    print('[INFO] Bilateral augmentation: resolved joint/body/ray maps and symmetric observation statistics.')
