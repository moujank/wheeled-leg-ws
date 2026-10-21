"""Run bounded no-gradient simulator probes in separate processes; preserve logs."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--suite', choices=['transfer', 'selection'], required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
output = root / 'reports/terrain_transfer_20261020'
output.mkdir(parents=True, exist_ok=True)
logs = root / 'logs/rsl_rl'
source = logs / 'wheeled_leg_terrains/2026-10-17_17-43-03/model_7999.pt'
cases = []
if args.suite == 'transfer':
    cases = [
        ('source_narrow_sym', source, ['--scene', 'terrain-flat', '--leg-clips', 'v1']),
        ('source_narrow_adapting', source, ['--scene', 'terrain-flat', '--leg-clips', 'v1', '--normalizer-updates']),
        ('source_wide_preserved_training', source, ['--scene', 'terrain-flat', '--normalizer-updates', '--preserve-count']),
        ('historical_initial_frozen', logs / 'wheeled_leg_terrains/2026-10-19_19-59-28/model_0.pt', ['--scene', 'terrain-flat']),
        ('v1_old_native', logs / 'wheeled_leg_v1/2026-10-17_10-52-09_v1_stable_baseline/model_7999.pt', ['--scene', 'v1', '--leg-clips', 'v1']),
        ('v1_old_terrain_flat', logs / 'wheeled_leg_v1/2026-10-17_10-52-09_v1_stable_baseline/model_7999.pt', ['--scene', 'terrain-flat', '--leg-clips', 'v1']),
        ('v1_current_native', logs / 'wheeled_leg_v1/2026-10-19_15-21-35_v1_stable_baseline/model_7999.pt', ['--scene', 'v1', '--leg-clips', 'v1']),
    ]
else:
    for label, run, step in [('scratch6000', '2026-10-20_09-29-55', 6000),
                             ('scratch7999', '2026-10-20_09-29-55', 7999),
                             ('transfer7000', '2026-10-19_19-59-28', 7000),
                             ('transfer7999', '2026-10-19_19-59-28', 7999)]:
        for level in (4, 7, 9):
            cases.append((f'{label}_row{level}', logs / f'wheeled_leg_terrains/{run}/model_{step}.pt',
                          ['--scene', 'terrain', '--level', str(level), '--num_envs', '192']))

status = []
for name, checkpoint, options in cases:
    command = [sys.executable, str(root / 'scripts/rsl_rl/probe_terrain_transfer.py'), '--headless',
               '--checkpoint', str(checkpoint), '--output', str(output / f'{name}.json'), *options]
    with (output / f'{name}.log').open('w') as stream:
        result = subprocess.run(command, cwd=root, stdout=stream, stderr=subprocess.STDOUT)
    status.append(dict(name=name, returncode=result.returncode, command=command))
    (output / f'{args.suite}_suite_status.json').write_text(json.dumps(status, indent=2))
    print(name, 'exit', result.returncode, flush=True)
    if result.returncode:
        print((output / f'{name}.log').read_text()[-3500:], flush=True)
        sys.exit(result.returncode)
