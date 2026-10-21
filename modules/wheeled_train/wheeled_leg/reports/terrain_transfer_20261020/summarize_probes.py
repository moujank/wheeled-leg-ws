"""Rank checkpoints by fixed first-episode outcomes, not training return."""
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parent
LABELS = ('scratch6000', 'scratch7999', 'transfer7000', 'transfer7999')
HARD = ('pyramid_stairs_inverted', 'rough', 'discrete_obstacles')
rows = []
ranking = []
for label in LABELS:
    cases = []
    for row in (4, 7, 9):
        f = ROOT / f'{label}_row{row}.json'
        if not f.exists():
            continue
        d = json.loads(f.read_text())
        types = d['first_episode_by_type']
        assert d['completed'] == d['num_envs']
        typescores = [types[t]['successes'] / types[t]['episodes'] for t in HARD]
        macro = sum(typescores) / len(HARD)
        result = dict(label=label, row=row, checkpoint=d['checkpoint'], hard_macro_success=macro,
                      failure_fraction=d['failure_fraction'],
                      total_success_fraction=sum(t['successes'] for t in types.values()) / d['num_envs'],
                      by_type=types)
        rows.append(result)
        cases.append(result)
    if len(cases) == 3:
        # Row 4 is a low-difficulty guard. Rows 7/9 have equal importance,
        # as do the three bottleneck types, irrespective of sampling proportions.
        challenging = [d for d in cases if d['row'] in (7, 9)]
        ranking.append(dict(label=label, checkpoint=cases[0]['checkpoint'],
            hard_macro_success=sum(d['hard_macro_success'] for d in challenging) / 2,
            mean_failure_fraction=sum(d['failure_fraction'] for d in challenging) / 2,
            row4_total_success=cases[0]['total_success_fraction']))
ranking.sort(key=lambda r: (-r['hard_macro_success'], r['mean_failure_fraction']))
result = dict(protocol='seed42, geometry43, 192 first episodes per row, .25m/s, rows4/7/9, stats frozen',
              rows=rows, ranking=ranking, complete=len(ranking) == 4)
if result['complete']:
    result['selected_checkpoint'] = ranking[0]['checkpoint']
(ROOT / 'selection_summary.json').write_text(json.dumps(result, indent=2))
if result['complete']:
    lines = [
        '# 固定难度 checkpoint 筛选', '',
        '同一地图 seed=43、重置 seed=42、0.25 m/s；每个 checkpoint 每行 192 个首回合。', '',
        '| 候选 | 第4行整体通过率 | 第7/9行重点类型平均通过率 | 第7/9行平均物理失败率 |',
        '|---|---:|---:|---:|',
    ]
    for item in ranking:
        lines.append(f"| {item['label']} | {item['row4_total_success']:.2%} | "
                     f"{item['hard_macro_success']:.2%} | {item['mean_failure_fraction']:.2%} |")
    lines += ['', '重点类型为下台阶、粗糙和离散障碍；类型与第7/9行等权。', '',
              '| 候选 | 行 | 下台阶成功/首回合 | 粗糙成功/首回合 | 离散障碍成功/首回合 | 整体物理失败率 |',
              '|---|---:|---:|---:|---:|---:|']
    for item in rows:
        counts = [f"{item['by_type'][t]['successes']}/{item['by_type'][t]['episodes']}" for t in HARD]
        lines.append(f"| {item['label']} | {item['row']} | " + ' | '.join(counts)
                     + f" | {item['failure_fraction']:.2%} |")
    lines += ['', '建议微调起点：', '', '```text', result['selected_checkpoint'], '```', '',
              '这只是四个代表性候选的单地图筛查，不是全部历史 checkpoint 的最优性证明。',
              '通过判据仍要求位移、时长、跟踪和姿态，不能保证经历随机地图的每个障碍。',
              '困难地形仍存在明显失败，当前模型尚未达到可靠困难地形通行要求。', '']
    (ROOT / 'selection_results.zh.md').write_text('\n'.join(lines))
print(json.dumps(ranking, indent=2))
for r in rows:
    print(r['label'], r['row'], f"hard macro {r['hard_macro_success']:.3f}",
          ', '.join(f"{k}:{v['successes']}/{v['episodes']}" for k,v in r['by_type'].items()))
