"""Bounded, no-gradient focus checkpoint review; one first episode per env."""
from pathlib import Path
import argparse
import json
import subprocess
import sys

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
RUN = ROOT / 'logs/rsl_rl/wheeled_leg_terrains/2026-10-21_11-14-05_low_speed_focus'
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--steps', type=int, nargs='+', default=[2000, 4000, 7999])
parser.add_argument('--rows', type=int, nargs='+', default=[4, 7, 9])
parser.add_argument('--reset-seed', type=int, default=42)
parser.add_argument('--map-seed', type=int, default=43)
parser.add_argument('--num-envs', type=int, default=192)
args = parser.parse_args()
status = []
for step in args.steps:
    for row in args.rows:
        name = f'focus{step}_row{row}_reset{args.reset_seed}_map{args.map_seed}'
        result_path = OUT / f'{name}.json'
        command = [sys.executable, 'scripts/rsl_rl/probe_terrain_transfer.py', '--headless',
                   '--checkpoint', str(RUN / f'model_{step}.pt'), '--output', str(result_path),
                   '--scene', 'terrain', '--level', str(row), '--speed', '.25', '--num_envs', str(args.num_envs),
                   '--seed', str(args.reset_seed), '--terrain-seed', str(args.map_seed), '--preserve-count']
        with (OUT / f'{name}.log').open('w') as log:
            result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        valid = result_path.is_file() and json.loads(result_path.read_text()).get('completed') == args.num_envs
        status.append(dict(name=name, returncode=result.returncode, valid=valid, command=command))
        (OUT / ('suite_' + '_'.join(map(str, args.steps)) + f'_r{args.reset_seed}_m{args.map_seed}_status.json')).write_text(json.dumps(status, indent=2))
        print(name, 'exit', result.returncode, flush=True)
        if result.returncode or not valid:
            print((OUT / f'{name}.log').read_text()[-4000:], flush=True)
            sys.exit(result.returncode or 1)
