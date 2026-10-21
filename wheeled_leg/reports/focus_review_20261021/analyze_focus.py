"""Reproducible scalar review of the completed focus experiment."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
RUN = ROOT / 'logs/rsl_rl/wheeled_leg_terrains/2026-10-21_11-14-05_low_speed_focus'
NAMES = ('flat', 'slope', 'slope_inverted', 'pyramid_stairs',
         'pyramid_stairs_inverted', 'rough', 'wave', 'discrete_obstacles')
ea = EventAccumulator(str(RUN), size_guidance={'scalars': 0})
ea.Reload()
data = {tag: np.array([(s.step, s.value) for s in ea.Scalars(tag)])
        for tag in ea.Tags()['scalars']}
np.savez_compressed(OUT / 'focus_scalars.npz', **data)
get = lambda tag: data[tag][:, 1]
windows = [(0, 200), (800, 1000), (1800, 2000), (3800, 4000),
           (5800, 6000), (7800, 8000)]
summary = {'run': str(RUN), 'windows': [],
           'notes': ['Termination logs average the latest outcome stored for each environment; they are not counts of ended episodes in this window.',
                     'Curriculum cumulative counters are averaged over collection steps before TensorBoard logging; their differences approximate window rates.',
                     'Train/mean_reward averages the most recent 100 ended episodes; the chart adds a separate 100-iteration smoothing.']}
for start, end in windows:
    mean = lambda tag: float(get(tag)[start:end].mean())
    delta = lambda tag: float(get(tag)[end - 1] - (get(tag)[start - 1] if start else 0))
    w = dict(start=start, end_exclusive=end,
             reward=mean('Train/mean_reward'),
             duration_s=mean('Train/mean_episode_length') * .008,
             terrain_row=mean('Curriculum/terrain_levels/mean_level'),
             tracking_error_mps=mean('Metrics/base_velocity/moving_tracking_error_mean'),
             stall_fraction=mean('Metrics/base_velocity/stall_fraction'),
             noise_std=mean('Policy/mean_noise_std'),
             value_loss=mean('Loss/value_function'),
             surrogate_loss=mean('Loss/surrogate'),
             symmetry_loss=mean('Loss/symmetry'),
             terminations={k: mean('Episode_Termination/' + k) for k in
                           ('time_out', 'base_contact', 'terrain_boundary', 'excessive_tilt')},
             by_type={}, reward_components={t.split('/', 1)[1]: mean(t)
                                           for t in data if t.startswith('Episode_Reward/')})
    w['physical_failure_log_fraction'] = w['terminations']['base_contact'] + w['terminations']['excessive_tilt']
    w['return_per_s_proxy'] = w['reward'] / w['duration_s']
    for name in NAMES:
        prefix = 'Curriculum/terrain_levels/' + name
        attempts, successes, top_attempts, top_successes = [
            delta(prefix + '_' + k) for k in ('attempts', 'successes', 'top_attempts', 'top_successes')]
        w['by_type'][name] = dict(
            mean_row=mean('Curriculum/terrain_distribution/' + name + '_mean_level'),
            attempts=attempts, successes=successes,
            success_rate=successes / attempts if attempts else None,
            top_attempts=top_attempts, top_successes=top_successes,
            top_success_rate=top_successes / top_attempts if top_attempts else None)
    summary['windows'].append(w)
reward, duration = get('Train/mean_reward'), get('Train/mean_episode_length') * .008
summary['late_reward_duration_correlation'] = float(np.corrcoef(reward[2000:], duration[2000:])[0, 1])
summary['peak_ma200_reward'] = float(np.convolve(reward, np.ones(200) / 200, 'valid').max())
summary['peak_ma200_ending_iteration'] = int(np.argmax(np.convolve(reward, np.ones(200) / 200, 'valid')) + 199)
summary['wall_time_seconds'] = ea.Scalars('Train/mean_reward')[-1].wall_time - ea.Scalars('Train/mean_reward')[0].wall_time
(OUT / 'curve_metrics.json').write_text(json.dumps(summary, indent=2))
fig, axes = plt.subplots(3, 2, figsize=(14, 12), constrained_layout=True)
ma = lambda a: np.convolve(a, np.ones(100) / 100, 'valid')
x = np.arange(99, len(reward))
for a, title, series in [
    (axes[0, 0], 'Episode return and duration (100-iteration mean)', [('return', reward), ('seconds x 5', duration * 5)]),
    (axes[0, 1], 'Return / duration (proxy, per second)', [('reward / seconds', reward / np.maximum(duration, .008))]),
    (axes[1, 0], 'Mean assigned terrain row (sampling, not success)',
     [(n, get('Curriculum/terrain_distribution/' + n + '_mean_level')) for n in NAMES]),
    (axes[1, 1], 'Latest per-env termination outcome (log snapshot)', [(n, get('Episode_Termination/' + n)) for n in
                                               ('time_out', 'terrain_boundary', 'base_contact', 'excessive_tilt')]),
    (axes[2, 0], 'Moving tracking error and stall fraction', [
        ('moving error (m/s)', get('Metrics/base_velocity/moving_tracking_error_mean')),
        ('stall fraction', get('Metrics/base_velocity/stall_fraction'))]),
    (axes[2, 1], 'Optimization diagnostics', [
        ('value loss', get('Loss/value_function')), ('symmetry loss', get('Loss/symmetry')),
        ('action noise std', get('Policy/mean_noise_std'))])]:
    for label, y in series:
        a.plot(x, ma(y), label=label, lw=1.2)
    a.set_title(title); a.set_xlabel('Learning iteration'); a.grid(alpha=.25)
    a.legend(fontsize=8)
fig.savefig(OUT / 'focus_training_review.png', dpi=160)
fig.savefig(OUT / 'focus_training_review.pdf')
print(json.dumps({k: v for k, v in summary.items() if k != 'windows'}, indent=2))
for w in summary['windows']:
    print(json.dumps({k: v for k, v in w.items() if k not in ('by_type', 'reward_components')}, indent=2))
    print(json.dumps(w['by_type'], indent=2))
