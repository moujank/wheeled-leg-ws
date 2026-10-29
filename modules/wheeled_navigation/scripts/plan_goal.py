"""Plan another goal on a saved scan map without starting Isaac Sim."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scan_nav.core import OccupancyMapper, path_length

p = argparse.ArgumentParser()
p.add_argument("--map", required=True, type=Path)
p.add_argument("--start", required=True, nargs=2, type=float)
p.add_argument("--goal", required=True, nargs=2, type=float)
p.add_argument("--radius", type=float, default=0.45)
p.add_argument("--output", type=Path)
a = p.parse_args()
mapper = OccupancyMapper.load(a.map)
try:
    path = mapper.plan(a.start, a.goal, a.radius)
except ValueError as exc:
    p.exit(2, f"Planning rejected: {exc}\n")
result = {"start": a.start, "goal": a.goal, "length_m": path_length(path), "path": path}
if a.output:
    a.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
print(json.dumps(result))
