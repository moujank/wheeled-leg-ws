"""Known geometry is used only to design a preset collection tour and audit it.

The final mapper and navigation planner never receive this geometry grid.
"""
import json
import math
from pathlib import Path
import numpy as np
from .core import GridSpec, OccupancyMapper

SPEC = GridSpec(.05, -4.9, -3.4, 196, 136)
MAP_SPEC = GridSpec(.025, -4.9, -3.4, 392, 272)


def load_geometry(path):
    rows=json.loads(Path(path).read_text(encoding='utf-8'))
    return [x for x in rows if x['max'][2] > .06 and x['min'][2] < 1.0
            and x['max'][2]-x['min'][2] > .015]


def clearance(x, y, geometry):
    # The floor boundary is part of the oracle; no route may leave the floor.
    edge=min(x+4.5,4.5-x,y+3,3-y)
    return min([edge]+[math.hypot(max(g['min'][0]-x,0,x-g['max'][0]),
                               max(g['min'][1]-y,0,y-g['max'][1])) for g in geometry])


def geometry_grid(geometry):
    oracle=OccupancyMapper(SPEC)
    oracle.observed[:]=True
    oracle.log_odds[:]=-4
    xx=SPEC.xmin+(np.arange(SPEC.width)+.5)*SPEC.resolution
    yy=SPEC.ymin+(np.arange(SPEC.height)+.5)*SPEC.resolution
    X,Y=np.meshgrid(xx,yy)
    occupied=(X < -4.5)|(X>4.5)|(Y < -3)|(Y>3)
    for g in geometry:
        occupied|=(X>=g['min'][0])&(X<=g['max'][0])&(Y>=g['min'][1])&(Y<=g['max'][1])
    oracle.log_odds[occupied]=4
    return oracle


def collection_tour(geometry, radius=.25):
    oracle=geometry_grid(geometry)
    # Named survey positions chosen from the existing room layout. Connecting
    # these positions is collection-route design, never the final navigation.
    candidates=[('起点',(.9,-1.4)),('卧室南侧',(-1.9,-1.15)),
                ('卧室西侧',(-1.95,1.55)),('休闲区',(-3.65,0.0)),
                ('卧室东侧',(.92,1.6)),('衣帽间南侧',(2.0,-1.55)),
                ('衣帽间中部',(2.0,.7)),('衣帽间北侧',(3.1,1.5)),
                ('衣帽间东侧',(3.4,-1.25))]
    start=candidates[0][1]
    accepted=[candidates[0]]
    rejected=[]
    tour=[start]
    for name,point in candidates[1:]:
        try:
            path=oracle.plan(tour[-1],point,radius)
            accepted.append((name,point))
            tour.extend(path[1:])
        except ValueError as exc:
            rejected.append({'name':name,'point':point,'reason':str(exc)})
    tour.extend(oracle.plan(tour[-1],start,radius)[1:])
    return tour,accepted,rejected


def sample_tour(path, step=.16):
    # Resample along cumulative distance, with a look-ahead heading. Root pose
    # setting is intentional in this collection baseline.
    points=np.asarray(path,dtype=float)
    distances=np.r_[0,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    _,unique=np.unique(distances,return_index=True)
    distances,points=distances[unique],points[unique]
    samples=np.arange(0,distances[-1]+1e-6,step)
    for value in samples:
        x,y=np.interp(value,distances,points[:,0]),np.interp(value,distances,points[:,1])
        ahead=min(value+.15,distances[-1])
        ax,ay=np.interp(ahead,distances,points[:,0]),np.interp(ahead,distances,points[:,1])
        yield (float(x),float(y),math.atan2(ay-y,ax-x))


def path_clearance(path, geometry):
    result=float('inf')
    for a,b in zip(path,path[1:]):
        n=max(1,math.ceil(math.dist(a,b)/.02))
        for i in range(n+1):
            t=i/n
            result=min(result,clearance(a[0]*(1-t)+b[0]*t,a[1]*(1-t)+b[1]*t,geometry))
    return result
