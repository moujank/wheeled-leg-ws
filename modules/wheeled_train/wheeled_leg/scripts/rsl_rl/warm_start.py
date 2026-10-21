"""Transfer a compatible PPO policy to a fresh run, including observation statistics."""

from pathlib import Path

import torch


_V1_OBSERVATION_DIMS = {
    "base_lin_vel": 3,
    "base_ang_vel": 3,
    "projected_gravity": 3,
    "velocity_commands": 3,
    "joint_pos": 4,
    "joint_vel": 6,
    "actions": 6,
    "body_contact": 7,
}


def _observation_column_map(env, group, source_width):
    """Map v1 input columns to target columns by observation term name.

    Terrain height scans are inserted into the target observation groups while
    the old v1 critic has a ``body_contact`` term. A raw prefix copy can wire
    old contact weights/statistics into height samples (or the reverse),
    depending on manager term ordering. Preserve the semantic term layout.
    """
    base = env.unwrapped if hasattr(env, "unwrapped") else env
    manager = getattr(base, "observation_manager", None)
    if manager is None or group not in manager.active_terms:
        return None

    source_terms = list(_V1_OBSERVATION_DIMS)
    if source_width == 28:
        source_terms = source_terms[:-1]
    elif source_width != 35:
        return None

    target_terms = manager.active_terms[group]
    target_dims = manager.group_obs_term_dim[group]
    target_offsets = {}
    offset = 0
    for name, shape in zip(target_terms, target_dims, strict=True):
        if len(shape) != 1:
            return None
        target_offsets[name] = (offset, shape[0])
        offset += shape[0]

    source_map = []
    source_offset = 0
    mapped_width = 0
    for name in source_terms:
        width = _V1_OBSERVATION_DIMS[name]
        if name in target_offsets and target_offsets[name][1] == width:
            source_start, _ = target_offsets[name]
            source_map.append((slice(source_offset, source_offset + width),
                               slice(source_start, source_start + width)))
            mapped_width += width
        source_offset += width
    if source_offset != source_width:
        return None
    if mapped_width != source_width:
        raise ValueError(
            f"Cannot align all {group} terms from the {source_width}-channel v1 checkpoint "
            f"to target terms {target_terms}."
        )
    return source_map


def _expand_state(source, target, expandable_keys, appended_observations, column_map=None):
    if source.keys() != target.keys():
        raise ValueError("Checkpoint state keys differ from the target network/normalizer.")
    result = {}
    for name, value in source.items():
        expected = target[name]
        if column_map and name in ("actor.0.weight", "critic.0.weight"):
            if value.ndim != 2 or expected.ndim != 2 or value.shape[0] != expected.shape[0]:
                raise ValueError(f"Incompatible checkpoint tensor {name}: {tuple(value.shape)} -> {tuple(expected.shape)}")
            result[name] = expected.clone()
            result[name].zero_()
            for source_slice, target_slice in column_map:
                if source_slice.stop <= value.shape[1] and target_slice.stop <= expected.shape[1]:
                    result[name][:, target_slice] = value[:, source_slice]
        elif column_map and name in {"_mean", "_var", "_std"}:
            if value.ndim != 2 or expected.ndim != 2 or value.shape[0] != expected.shape[0]:
                raise ValueError(f"Incompatible checkpoint tensor {name}: {tuple(value.shape)} -> {tuple(expected.shape)}")
            result[name] = expected.clone()
            for source_slice, target_slice in column_map:
                if source_slice.stop <= value.shape[1] and target_slice.stop <= expected.shape[1]:
                    result[name][:, target_slice] = value[:, source_slice]
        elif value.shape == expected.shape:
            result[name] = value
        elif (name in expandable_keys and value.ndim == expected.ndim
              and value.shape[:-1] == expected.shape[:-1]
              and expected.shape[-1] == value.shape[-1] + appended_observations
              and appended_observations > 0):
            # New input weights start at zero; normalizer mean/variance use their defaults.
            result[name] = expected.clone()
            if name.endswith(".weight"):
                result[name].zero_()
            result[name][..., :value.shape[-1]] = value
        else:
            raise ValueError(f"Incompatible checkpoint tensor {name}: {tuple(value.shape)} -> {tuple(expected.shape)}")
        if not torch.isfinite(result[name]).all():
            raise ValueError(f"Checkpoint contains non-finite values: {name}")
    return result


def warm_start_runner(runner, checkpoint_path, appended_observations=0, env=None,
                      preserve_normalizer_count=False):
    """Load weights and statistics; keep the new optimizer, learning rate and iteration.

    Only same-shape networks or an explicitly allowed number of appended observation
    columns are accepted. Callers must preserve the source observation/action ordering.
    This supports the v1/terrain-balance -> obstacle/jump tasks in this project.
    """
    path = Path(checkpoint_path).expanduser().resolve(strict=True)
    saved = torch.load(path, map_location=runner.device, weights_only=False)
    source_policy = saved["model_state_dict"]
    target_policy = runner.alg.policy.state_dict()
    actor_map = _observation_column_map(env, "policy", source_policy["actor.0.weight"].shape[1]) if env else None
    critic_map = _observation_column_map(env, "critic", source_policy["critic.0.weight"].shape[1]) if env else None
    # Keep the original prefix-expansion fallback for callers without an env.
    policy_state = _expand_state(
        source_policy, target_policy, {"actor.0.weight", "critic.0.weight"}, appended_observations,
        column_map=None,
    )
    if actor_map is not None or critic_map is not None:
        for group, mapping, key in (("policy", actor_map, "actor.0.weight"),
                                    ("critic", critic_map, "critic.0.weight")):
            if mapping is None:
                continue
            source_weight = source_policy[key]
            target_weight = target_policy[key].clone().zero_()
            for source_slice, target_slice in mapping:
                target_weight[:, target_slice] = source_weight[:, source_slice]
            policy_state[key] = target_weight
    normalizers = []
    if runner.empirical_normalization:
        for key, module in (("obs_norm_state_dict", runner.obs_normalizer),
                            ("privileged_obs_norm_state_dict", runner.privileged_obs_normalizer)):
            if key not in saved:
                raise ValueError(f"Normalized target requires checkpoint statistics: {key}")
            group = "policy" if key == "obs_norm_state_dict" else "critic"
            mapping = actor_map if group == "policy" else critic_map
            normalizers.append((module, _expand_state(
                saved[key], module.state_dict(), {"_mean", "_var", "_std"}, appended_observations,
                column_map=mapping,
            )))
    # Validate every tensor before modifying the runner.
    runner.alg.policy.load_state_dict(policy_state)
    for module, state in normalizers:
        module.load_state_dict(state)
        # Preserve mature controller statistics when requested. Other callers
        # retain the existing bounded-history adaptation behavior.
        if not preserve_normalizer_count:
            module.count.clamp_(max=10000)
    runner.current_learning_iteration = 0
    if actor_map is not None or critic_map is not None:
        print(
            f"[INFO] Warm-start loaded checkpoint iteration {saved.get('iter', 'unknown')}: "
            f"actor {source_policy['actor.0.weight'].shape[1]} -> "
            f"{target_policy['actor.0.weight'].shape[1]} inputs, "
            f"critic {source_policy['critic.0.weight'].shape[1]} -> "
            f"{target_policy['critic.0.weight'].shape[1]} inputs. "
            "Observation terms aligned by name; new terrain/jump input weights remain zero-initialized."
        )
    return str(path)
