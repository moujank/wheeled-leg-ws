"""Compare bounded checkpoint probes with the actual initialization baseline."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent
OLD = OUT.parent / 'terrain_transfer_20261020'
HARD = ('pyramid_stairs_inverted', 'rough', 'discrete_obstacles')
cases = []
for label, paths in [('historical_scratch6000', [OLD / f'scratch6000_row{r}.json' for r in (4, 7, 9)]),
                     ('source7000', [OLD / f'transfer7000_row{r}.json' for r in (4, 7, 9)]),
                     *[(f'focus{s}', [OUT / f'focus{s}_row{r}_reset42_map43.json' for r in (4, 7, 9)])
                       for s in (2000, 4000, 6000, 7999)]]:
    results = []
    for f in paths:
        if not f.exists():
            continue
        d = json.loads(f.read_text())
        assert d['completed'] == d['num_envs'] == 192
        assert d['zero_update_action_max_error'] < 1e-5
        types = d['first_episode_by_type']
        assert sum(v['episodes'] for v in types.values()) == 192
        results.append(dict(label=label, row=d['level'], checkpoint=d['checkpoint'],
                            source_file=str(f), by_type=types,
                            overall_success=sum(v['successes'] for v in types.values()) / 192,
                            hard_macro_success=np.mean([types[t]['successes'] / types[t]['episodes'] for t in HARD]),
                            physical_failure=d['failure_fraction'],
                            tracking_error=d['tracking_error_mps']))
    if len(results) == 3:
        hard = [r for r in results if r['row'] in (7, 9)]
        cases.append(dict(label=label, checkpoint=results[0]['checkpoint'], rows=results,
                          row4_success=results[0]['overall_success'],
                          hard_macro_success=float(np.mean([r['hard_macro_success'] for r in hard])),
                          hard_physical_failure=float(np.mean([r['physical_failure'] for r in hard]))))
ranking = sorted(cases, key=lambda c: (-c['hard_macro_success'], c['hard_physical_failure']))
payload = dict(protocol='192 first episodes/row, reset42, geometry43, fixed .25m/s, frozen normalization',
               candidates=cases, ranking=[c['label'] for c in ranking],
               selected_checkpoint=ranking[0]['checkpoint'] if ranking else None,
               complete=len(cases) == 6)
(OUT / 'evaluation_summary.json').write_text(json.dumps(payload, indent=2))
lines = ['# 固定地形评估', '',
         'reset seed42，geometry seed43，0.25m/s，每个模型每行192个首回合；关闭动作/观测噪声、保留物理随机化与源观测统计。', '',
         '| 模型 | 第4行整体通过 | 第7/9行重点类型等权通过 | 第7/9行整体物理失败 |',
         '|---|---:|---:|---:|']
for c in cases:
    lines.append(f"| {c['label']} | {c['row4_success']:.2%} | {c['hard_macro_success']:.2%} | {c['hard_physical_failure']:.2%} |")
lines += ['', '重点类型：下台阶、粗糙、离散障碍。高行与类型等权，避免采样权重改变排名。', '',
          '| 模型 | 行 | 下台阶成功 | 粗糙成功 | 离散障碍成功 | 所有类型物理失败 |',
          '|---|---:|---:|---:|---:|---:|']
for c in cases:
    for r in c['rows']:
        cells = [f"{r['by_type'][t]['successes']}/{r['by_type'][t]['episodes']}" for t in HARD]
        lines.append(f"| {c['label']} | {r['row']} | " + ' | '.join(cells) + f" | {r['physical_failure']:.2%} |")
lines += ['', '这些是代表模型在单一地图种子下的筛查结果，不能证明所有 checkpoint 的最优性或真实机器人可靠性。',
          '本轮 focus 训练的地形种子也是43；本表为固定条件复测，不属于未见地图泛化评估。',
          'historical_scratch6000 与 source7000 复用上一轮同配置固定评估；当时 count=10000 但归一化处于 eval 模式，统计数值不变，count 不影响动作。',
          '训练课程通过误差阈值0.15m/s；本表固定评估更严格，为0.10m/s。', '']
(OUT / 'evaluation_results.zh.md').write_text('\n'.join(lines))
if cases:
    fig, axes = plt.subplots(1, 3, figsize=(14, 4), constrained_layout=True)
    for ax, metric, title in zip(axes, ('row4_success', 'hard_macro_success', 'hard_physical_failure'),
                                 ('Row 4: overall success', 'Rows 7/9: hard-type macro success', 'Rows 7/9: overall physical failure')):
        ax.bar([c['label'] for c in cases], [100 * c[metric] for c in cases])
        ax.set_ylim(0, 100); ax.set_ylabel('%'); ax.set_title(title); ax.tick_params(axis='x', rotation=30)
        ax.grid(axis='y', alpha=.2)
    fig.savefig(OUT / 'fixed_evaluation.png', dpi=160)
    fig.savefig(OUT / 'fixed_evaluation.pdf')
print('\n'.join(lines))
