"""Rebuild the comparison from cached TensorBoard scalar exports."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
RUNS = [('scratch', '2026-10-20_09-29-55'), ('transfer', '2026-10-19_19-59-28')]
fig, axes = plt.subplots(3, 2, figsize=(13, 12), constrained_layout=True)
summary = {}
for label, run in RUNS:
    data = np.load(ROOT / f'{run}_scalars.npz')
    get = lambda tag: data[tag][:, 1]
    reward = get('Train/mean_reward')
    duration = get('Train/mean_episode_length') * .008
    level = get('Curriculum/terrain_levels/mean_level')
    fall = get('Episode_Termination/base_contact') + get('Episode_Termination/excessive_tilt')
    error = get('Metrics/base_velocity/moving_tracking_error_mean')
    density = reward / np.maximum(duration, .008)
    x = data['Train/mean_reward'][:, 0]
    for ax, values, title in zip(axes.flat,
            [reward, duration, density, level, fall, error],
            ['Episode return (100-iteration average)', 'Episode duration (s)',
             'Return / duration (proxy, per second)', 'Mean assigned terrain row',
             'Physical failure termination fraction', 'Moving velocity tracking error (m/s)']):
        smooth = np.convolve(values, np.ones(100) / 100, mode='valid')
        ax.plot(x[99:], smooth, label=label, linewidth=1.2)
        ax.set_title(title)
        ax.set_xlabel('Learning iteration')
        ax.grid(alpha=.25)
    ma20 = np.convolve(reward, np.ones(20) / 20, mode='valid')
    hit = np.flatnonzero(ma20 >= 70)
    summary[label] = dict(run=run, reward70_ma20_iteration=int(x[hit[0] + 19]) if len(hit) else None,
        last200_reward=float(reward[-200:].mean()),
        last200_duration_s=float(duration[-200:].mean()),
        last200_return_per_s_proxy=float(reward[-200:].mean() / duration[-200:].mean()),
        last200_terrain_row=float(level[-200:].mean()),
        last200_physical_failure_fraction=float(fall[-200:].mean()),
        initial_noise_std=float(get('Policy/mean_noise_std')[0]),
        late_reward_duration_correlation=float(np.corrcoef(reward[5000:], duration[5000:])[0, 1]),
        last200_reward_components={tag.removeprefix('Episode_Reward/'): float(get(tag)[-200:].mean())
                                   for tag in data.files if tag.startswith('Episode_Reward/')})
axes[0, 0].legend()
fig.savefig(ROOT / 'training_comparison.png', dpi=160)
fig.savefig(ROOT / 'training_comparison.pdf')
(ROOT / 'curve_metrics.json').write_text(json.dumps(summary, indent=2))
print(json.dumps({k: {a:b for a,b in v.items() if a != 'last200_reward_components'} for k,v in summary.items()}, indent=2))
