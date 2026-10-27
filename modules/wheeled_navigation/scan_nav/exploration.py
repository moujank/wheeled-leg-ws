"""Online frontier exploration. This module receives measured grids only.

There is deliberately no scene loader, mesh query, itinerary or true map input.
Unknown cells remain impassable; a frontier goal is placed back inside measured
free space so that the complete robot can reach it before taking another scan.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
import numpy as np
from scipy import ndimage as ndi
from .core import GridSpec, OccupancyMapper

ROBOT_RADIUS = 0.38  # encloses the actual bent wheeled_arm_v2 mesh in XY


class ExplorationMap(OccupancyMapper):
    """Known-free centres with full-radius obstacle clearance.

    Unknown centre cells cannot be traversed. Unlike the offline strict mapper,
    unknown cells are not themselves inflated; otherwise lidar occlusion beside
    the current footprint can prevent any first exploration step. New returns
    are checked before each velocity step. This is explicitly a partial-map
    exploration policy, not a claim that unobserved footprint edges are empty.
    """
    def blocked(self,radius=ROBOT_RADIUS):
        if not math.isfinite(radius) or radius<0:raise ValueError('Invalid radius')
        # Occupied cells represent finite squares; add their half diagonal.
        # Commands additionally check the swept circle against the cell boxes.
        reach=radius+self.spec.resolution*math.sqrt(2.)/2.
        occupied=self.occupied()
        distance=ndi.distance_transform_edt(~occupied)*self.spec.resolution
        blocked=(~self.free()) | (distance<=reach)
        n=math.ceil(reach/self.spec.resolution)
        blocked[:n,:]=blocked[-n:,:]=True;blocked[:,:n]=blocked[:,-n:]=True
        return blocked


@dataclass
class FrontierGoal:
    position: tuple
    path: list
    information_cells: int
    travel_distance: float
    score: float
    yaw: float | None = None


def from_occupancy(data, resolution, origin):
    """ROS OccupancyGrid adapter; no source geometry is accepted."""
    data = np.asarray(data)
    m = ExplorationMap(GridSpec(float(resolution), float(origin[0]),
                                float(origin[1]), data.shape[1], data.shape[0]))
    m.observed[:] = data >= 0
    m.log_odds[data == 0] = -3
    m.log_odds[data >= 50] = 3
    return m


class FrontierExplorer:
    def __init__(self, radius=ROBOT_RADIUS, min_travel=0.45, polygon=False):
        self.radius = radius
        self.min_travel = min_travel
        self.attempts = []
        self.rejected = []
        self.last_diagnostics = {}
        self.polygon=polygon

    def record(self, goal, success, seconds=0.):
        self.attempts.append((tuple(goal), bool(success), float(seconds)))

    def choose(self, mapper, pose, seconds=0.):
        free = mapper.free()
        unknown = ~mapper.observed
        # Invalid intermediate probabilities also stay blocked.
        safe = ~mapper.blocked(self.radius)
        c, r = mapper.cell(*pose[:2])
        if not mapper.inside(c, r) or not safe[r, c]:
            self.last_diagnostics = {"reason": "robot_outside_measured_safe_space"}
            return None
        structure = ndi.generate_binary_structure(2, 1)
        seed=np.zeros_like(free);seed[r,c]=True
        reachable=ndi.binary_propagation(seed,mask=safe,structure=structure)
        frontier = free & ndi.binary_dilation(unknown, structure=structure)
        # A target must be within sensor reach of a frontier, but leave enough
        # room for the complete robot footprint on the observed side.
        frontier_distance = ndi.distance_transform_edt(~frontier)*mapper.spec.resolution
        clearance = ndi.distance_transform_edt(free)*mapper.spec.resolution
        candidates = reachable & (frontier_distance < self.radius+0.22)
        rows, cols = np.nonzero(candidates)
        self.last_diagnostics = {"frontier_cells": int(frontier.sum()),
                                 "reachable_cells": int(reachable.sum()),
                                 "candidate_cells": len(rows)}
        if not len(rows) or not frontier.any():
            return None
        # Information estimate is computed on UNKNOWN map cells only. It is a
        # heuristic, not a raycast through an oracle map or known floor plan.
        window = max(3, int(1.4/mapper.spec.resolution))
        gain = ndi.uniform_filter(unknown.astype(float), size=2*window+1,
                                  mode="constant") * (2*window+1)**2
        # Sample candidates on a metric lattice, not every individual grid cell.
        stride = max(1, round(.18/mapper.spec.resolution))
        # Subsample the candidate list, not fixed grid columns: a narrow safe
        # corridor must not disappear just because it falls between lattice lines.
        rows, cols = rows[::stride], cols[::stride]
        ranked = []
        for row, col in zip(rows, cols):
            xy = mapper.world(col, row)
            distance = math.dist(xy, pose[:2])
            if distance < self.min_travel:
                continue
            # Retrying a failed location is allowed only after the map changes
            # substantially or a cooldown. Successful viewpoints are avoided.
            penalty = 0.
            skip = False
            for old, success, when in self.attempts:
                near = math.dist(xy, old)
                if not success and near < .65 and seconds-when < 90:
                    skip = True
                    break
                if success:
                    penalty += max(0., 1.-near/1.0)*3.0
            if skip or gain[row,col] < 10:
                continue
            score = math.log1p(gain[row,col])/(.8+distance) - penalty
            score += min(clearance[row,col], 1.)*.1
            if score<=.1:continue
            ranked.append((score, xy, int(gain[row,col])))
        # Spatial diversity prevents a corridor full of nearby candidate cells
        # from excluding every doorway/other frontier from the shortlist.
        shortlist=[];buckets=set()
        for score,xy,info in sorted(ranked,reverse=True):
            bucket=(math.floor(xy[0]/.45),math.floor(xy[1]/.45))
            if bucket in buckets:continue
            buckets.add(bucket);shortlist.append((score,xy,info))
            if len(shortlist)>=45:break
        from .footprint import feasible_yaws,visible_unknown_gain
        choices=[]
        rejects={'path':0,'footprint':0,'gain':0,'novelty':0}
        for proxy, xy, info in shortlist:
            try:
                path = mapper.plan(pose[:2], xy, self.radius)
            except ValueError:
                rejects['path']+=1
                continue
            length = sum(math.dist(a,b) for a,b in zip(path,path[1:]))
            approach=path[0]
            for q in reversed(path):
                if math.dist(q,xy)>=.6:approach=q;break
            preferred=math.atan2(xy[1]-approach[1],xy[0]-approach[0])
            heading=None
            if self.polygon:
                headings=feasible_yaws(mapper,xy,preferred)
                if not headings:
                    rejects['footprint']+=1;continue
                heading=headings[0]
            gain=visible_unknown_gain(mapper,xy)
            if gain<.12:
                rejects['gain']+=1;continue
            visited=[math.dist(xy,old) for old,success,_ in self.attempts if success]
            novelty=min(1.,min(visited)/.85)**2 if visited else 1.
            score=gain*novelty/(.8+.3*length)
            if score<.025:
                rejects['novelty']+=1;continue
            choices.append(FrontierGoal(xy,path,int(gain/mapper.spec.resolution**2),length,score,heading))
        self.last_diagnostics['viewpoint_rejections']=rejects
        if choices:
            best=max(choices,key=lambda q:q.score)
            self.last_diagnostics.update(expected_visible_unknown_area_m2=best.information_cells*mapper.spec.resolution**2,
                                         scored_viewpoints=len(choices),polygon_goal_check=self.polygon)
            return best
        return None


def velocity_to_path(pose, path, speed=.30, yaw_rate=.55, lookahead=.14, heading_gate=.5):
    """Pure pursuit of a CURRENT path, returning vx/wz, never a new pose."""
    if not path or math.dist(pose[:2],path[-1]) < .16:
        return (0.,0.), True
    points = np.asarray(path)
    nearest = int(np.argmin(np.linalg.norm(points-np.asarray(pose[:2]),axis=1)))
    target = points[-1]
    for q in points[nearest:]:
        if math.dist(pose[:2],q) >= lookahead:
            target=q
            break
    error = math.atan2(math.sin(math.atan2(target[1]-pose[1],target[0]-pose[0])-pose[2]),
                       math.cos(math.atan2(target[1]-pose[1],target[0]-pose[0])-pose[2]))
    wz = float(np.clip(2.5*error,-yaw_rate,yaw_rate))
    vx = speed*max(0.,math.cos(error)) if abs(error)<heading_gate else 0.
    vx *= min(1., math.dist(pose[:2],path[-1])/.35)
    return (vx,wz), False


def integrate_velocity(pose, command, dt):
    """Kinematic actuator shared by both stages; no destination is accepted."""
    x,y,yaw = pose
    vx,wz = command
    mid = yaw+.5*wz*dt
    return (x+vx*math.cos(mid)*dt, y+vx*math.sin(mid)*dt,
            math.atan2(math.sin(yaw+wz*dt),math.cos(yaw+wz*dt)))


def map_stop(mapper,pose,command,radius=ROBOT_RADIUS):
    """Continuous short-horizon circle-vs-measured-cell check for the actuator."""
    if abs(command[0])<1e-6:return command,False
    s=mapper.spec
    for t in (.1,.2,.3,.4):
        predicted=integrate_velocity(pose,command,t)
        c,r=mapper.cell(*predicted[:2])
        if not mapper.inside(c,r) or not mapper.free()[r,c]:return (0.,command[1]),True
        n=math.ceil((radius+.1)/s.resolution)
        y0,y1=max(0,r-n),min(s.height,r+n+1);x0,x1=max(0,c-n),min(s.width,c+n+1)
        rr,cc=np.nonzero(mapper.occupied()[y0:y1,x0:x1])
        if not len(rr):continue
        lo=np.c_[s.xmin+(cc+x0)*s.resolution,s.ymin+(rr+y0)*s.resolution]
        hi=lo+s.resolution
        delta=np.maximum(np.maximum(lo-np.asarray(predicted[:2]),np.asarray(predicted[:2])-hi),0)
        if np.min(np.linalg.norm(delta,axis=1))<radius+.025:return (0.,command[1]),True
    return command,False


def measured_stop(command, pose, sensor_pose, angles, ranges, radius=ROBOT_RADIUS):
    """Stop translation if measured returns intersect its short swept circle."""
    vx,wz = command
    if abs(vx)<1e-6:
        return (vx,wz), False
    angles,ranges = np.asarray(angles),np.asarray(ranges)
    good=np.isfinite(ranges)&(ranges>=.15)&(ranges<19.99)
    points=np.c_[sensor_pose[0]+ranges[good]*np.cos(angles[good]+sensor_pose[2]),
                 sensor_pose[1]+ranges[good]*np.sin(angles[good]+sensor_pose[2])]
    # Measured points only. No PhysX overlap query, mesh bounds or floor map.
    for t in (.0,.2,.4,.6):
        predicted=integrate_velocity(pose,(vx,wz),t)
        if len(points) and np.min(np.linalg.norm(points-np.asarray(predicted[:2]),axis=1)) < radius+.075:
            return (0.,wz), True
    return (vx,wz), False
