"""Measured wheeled_arm_v2 XY envelope, with explicit orientation checks."""
import math
import numpy as np

# Prepared bent-leg/folded-arm asset world bounds relative to base_footprint.
# This contains the entire model; the robot is not scaled or artificially shrunk.
ROBOT_POLYGON=[[-.231726,-.264291],[.268151,-.264291],[.268151,.264315],[-.231726,.264315]]
PADDING=.01
INSCRIBED_RADIUS=.23


def pose_is_free(mapper,xy,yaw,require_known=True,padding=PADDING):
    """Polygon vs occupied finite grid squares via separating-axis tests."""
    s=mapper.spec
    xmin=min(p[0] for p in ROBOT_POLYGON)-padding
    xmax=max(p[0] for p in ROBOT_POLYGON)+padding
    ymin=min(p[1] for p in ROBOT_POLYGON)-padding
    ymax=max(p[1] for p in ROBOT_POLYGON)+padding
    local_centre=np.array([(xmin+xmax)/2,(ymin+ymax)/2])
    half=np.array([(xmax-xmin)/2,(ymax-ymin)/2])
    cs,sn=math.cos(yaw),math.sin(yaw)
    rotation=np.array([[cs,-sn],[sn,cs]])
    centre=np.asarray(xy)+rotation@local_centre
    extent=np.abs(rotation)@half+s.resolution/2
    c0,r0=mapper.cell(*(centre-extent));c1,r1=mapper.cell(*(centre+extent))
    if not mapper.inside(c0,r0) or not mapper.inside(c1,r1):return False
    rr,cc=np.indices((r1-r0+1,c1-c0+1));rr+=r0;cc+=c0
    cell_centres=np.c_[s.xmin+(cc.ravel()+.5)*s.resolution,s.ymin+(rr.ravel()+.5)*s.resolution]
    delta=cell_centres-centre
    # Four SAT axes: the two rectangle axes and the two cell-square axes.
    hit=np.all(np.abs(delta)<=extent,axis=1)
    projected=delta@rotation
    cell_projection=s.resolution/2*(np.abs(rotation).sum(axis=0))
    hit &= np.all(np.abs(projected)<=half+cell_projection,axis=1)
    blocked=~mapper.free() if require_known else mapper.occupied()
    return not np.any(blocked[rr.ravel()[hit],cc.ravel()[hit]])


def feasible_yaws(mapper,xy,preferred=0.,bins=32):
    headings=[i*2*math.pi/bins for i in range(bins)]
    headings.sort(key=lambda a:abs(math.atan2(math.sin(a-preferred),math.cos(a-preferred))))
    return [a for a in headings if pose_is_free(mapper,xy,a)]


def measured_polygon_stop(command,angles,ranges,mount_xy=(.16,0.)):
    """Local swept rectangle vs measured returns; no simulator pose is needed."""
    from .exploration import integrate_velocity
    angles=np.asarray(angles);ranges=np.asarray(ranges)
    good=np.isfinite(ranges)&(ranges>=.15)&(ranges<7.99)
    points=np.c_[ranges[good]*np.cos(angles[good]),ranges[good]*np.sin(angles[good])]+mount_xy
    pad=.012
    for t in (.0,.1,.2,.3,.4):
        predicted=integrate_velocity((0.,0.,0.),command,t)
        cs,sn=math.cos(predicted[2]),math.sin(predicted[2])
        rotation=np.array([[cs,-sn],[sn,cs]])
        local=(points-np.asarray(predicted[:2]))@rotation
        hit=(local[:,0]>-.231726-pad)&(local[:,0]<.268151+pad)&(local[:,1]>-.264291-pad)&(local[:,1]<.264315+pad)
        if hit.any():return (0.,0.),True
    return command,False


def visible_unknown_gain(mapper,xy,max_range=3.,beams=72):
    """Expected unknown area from rays in the MEASURED map, stopped by walls.

    Unknown transmission decays as rays enter unseen cells; no hidden geometry
    or labels are queried. This heuristic cannot guarantee the unseen space is
    empty. Already known cells contribute no information gain.
    """
    from scipy import ndimage as ndi
    s=mapper.spec;unknown=~mapper.observed
    wall=ndi.binary_dilation(mapper.occupied(),iterations=1)
    angles=np.arange(beams)*2*math.pi/beams
    steps=np.arange(max(.1,s.resolution),max_range,s.resolution*1.5)
    cols=np.floor((xy[0]+np.cos(angles[:,None])*steps-s.xmin)/s.resolution).astype(int)
    rows=np.floor((xy[1]+np.sin(angles[:,None])*steps-s.ymin)/s.resolution).astype(int)
    inside=(cols>=0)&(cols<s.width)&(rows>=0)&(rows<s.height)
    safe_c=np.clip(cols,0,s.width-1);safe_r=np.clip(rows,0,s.height-1)
    blocked=(~inside)|wall[safe_r,safe_c]
    visible=np.cumsum(blocked,axis=1)==0
    novel=unknown[safe_r,safe_c]&visible
    attenuation=np.power(.97,np.cumsum(novel,axis=1))
    flat=safe_r*s.width+safe_c
    weights=np.zeros(s.width*s.height,dtype=np.float32)
    np.maximum.at(weights,flat[novel],attenuation[novel])
    # Each ray samples a narrow angular wedge; account for the increasing beam
    # spacing, with a cap to keep far unknown area from dominating the score.
    wedge=np.minimum(3.,steps*2*math.pi/beams/s.resolution)
    weighted=np.zeros_like(weights)
    np.maximum.at(weighted,flat[novel],(attenuation*wedge)[novel])
    return float(weighted.sum()*s.resolution**2)
