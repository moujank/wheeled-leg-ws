"""Rebuild and check a saved scan experiment without starting Isaac Sim."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scan_nav.core import OccupancyMapper
from scan_nav.code_room import load_geometry,path_clearance

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('output',type=Path)
p.add_argument('--geometry',type=Path,required=True)
a=p.parse_args()
m=json.loads((a.output/'metrics.json').read_text(encoding='utf-8'))
assert m['status']=='passed',m
saved=OccupancyMapper.load(a.output)
data=np.load(a.output/'scans.npz',allow_pickle=False)
raw=data['raw_depth_uint16']
assert raw.dtype==np.uint16 and raw.shape==(m['scan_count'],4,360),raw.shape
r=raw.astype(float)*float(data['max_range'])/65535
valid=np.where(r>=.15,r,np.inf)
fused=valid.min(axis=1);fused[np.all(r<.15,axis=1)]=np.nan
np.testing.assert_allclose(fused,data['ranges'],atol=0,rtol=0,equal_nan=True)
assert np.max(np.linalg.norm(np.diff(data['poses'][:,:2],axis=0),axis=1))<=m['collection_step_m']+1e-6
assert np.median(np.abs(np.diff(fused,axis=0)))>0,'No scan changes along collection route'
rebuilt=OccupancyMapper(saved.spec)
for pose,scan in zip(data['poses'],data['ranges']):rebuilt.integrate(pose,data['angles'],scan,20)
np.testing.assert_array_equal(rebuilt.observed,saved.observed)
np.testing.assert_array_equal(rebuilt.log_odds,saved.log_odds)
assert saved.observed.any() and (~saved.observed).any()
geometry=load_geometry(a.geometry)
blocked=saved.blocked(m['planning_disk_radius_m'])
for case in m['planning_cases']:
    route=rebuilt.plan(case['start'],case['goal'],m['planning_disk_radius_m'])
    np.testing.assert_allclose(route,case['path'],atol=0,rtol=0)
    assert path_clearance(route,geometry)>=m['planning_disk_radius_m']
    for x,y in route:
        c,row=saved.cell(x,y);assert not blocked[row,c]
for case in m['negative_goal_tests']:
    try:rebuilt.plan(m['planning_cases'][0]['start'],case['point'],m['planning_disk_radius_m'])
    except ValueError:pass
    else:raise AssertionError(case)
result=dict(status='passed',raw_measurements=int(raw.size),
    rebuilt_map_exact_match=True,paths_reproduced=len(m['planning_cases']),
    rejected_goals=len(m['negative_goal_tests']),unknown_cells_preserved=int((~saved.observed).sum()))
(a.output/'recording_validation.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result))
