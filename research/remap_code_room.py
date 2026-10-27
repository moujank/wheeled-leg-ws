import sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'modules/wheeled_navigation'))
from scan_nav.core import GridSpec,OccupancyMapper
from scan_nav.code_room import load_geometry,path_clearance
out=root/'modules/wheeled_navigation/artifacts/code_as_room_demo'
d=np.load(out/'scans.npz');g=load_geometry(root/'research/scene_audit/code_as_room_geometry/code_as_room_colliders.json')
for res in [.025]:
    mapper=OccupancyMapper(GridSpec(res,-4.9,-3.4,round(9.8/res),round(6.8/res)))
    for pose,ranges in zip(d['poses'],d['ranges']):mapper.integrate(pose,d['angles'],ranges,20)
    for radius in [.25,.27,.30]:
        try:
            path=mapper.plan((.9,-1.4),(3.1,1.5),radius)
            print('PASS',res,radius,'gap',path_clearance(path,g))
        except ValueError as e: print('FAIL',res,radius,str(e))
