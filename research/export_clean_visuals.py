"""Re-render recorded scientific data as tightly cropped transparent assets.

No image synthesis or background-color removal is used: all plot furniture is
disabled before rendering, preserving white scene points and free-space cells.
"""
from pathlib import Path
import json,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
NAV=ROOT/'modules/wheeled_navigation'
sys.path.insert(0,str(NAV))
from scan_nav.core import OccupancyMapper
OUT=NAV/'artifacts/clean_visuals';OUT.mkdir(exist_ok=True)
RUN=NAV/'artifacts/autonomy_ros_no_table_v2'

def trim_alpha(path,padding=12):
    image=Image.open(path).convert('RGBA')
    box=image.getchannel('A').getbbox()
    assert box is not None
    image=image.crop(box)
    result=Image.new('RGBA',(image.width+padding*2,image.height+padding*2),(0,0,0,0))
    result.paste(image,(padding,padding))
    result.save(path)
    alpha=np.asarray(result.getchannel('A'))
    assert alpha[0,0]==0 and alpha[-1,-1]==0
    assert np.any(alpha==255) and np.any(alpha==0)
    return {'path':str(path),'width':result.width,'height':result.height,
        'mode':result.mode,'transparent_pixel_fraction':float((alpha==0).mean())}

with np.load(RUN/'reconstruction/native_rgb_colors.npz') as cloud:
    points=cloud['points'];colors=cloud['colors']
# Preserve the previous useful cutaway, removing only its axes, panes and text.
keep=~((points[:,1]<-3.5)&(points[:,2]>.3))
fig=plt.figure(figsize=(12,10),dpi=240,facecolor='none')
ax=fig.add_axes([0,0,1,1],projection='3d',facecolor='none')
ax.scatter(*points[keep].T,c=colors[keep]/255.,s=.7,depthshade=False,linewidths=0)
ax.set(xlim=(-4.7,4.7),ylim=(-3.9,3.9),zlim=(0,3))
ax.set_box_aspect((9,7.3,3));ax.view_init(elev=42,azim=-68)
ax.set_axis_off();ax.grid(False)
cloud_path=OUT/'pointcloud_rgb_transparent.png'
fig.savefig(cloud_path,transparent=True,dpi=240,pad_inches=0);plt.close(fig)
results=[trim_alpha(cloud_path)]

def trajectory(run,name):
    motion=json.loads((run/'motion.json').read_text())
    q=np.array([row['truth'] for row in motion])
    mapper=OccupancyMapper.load(run/'ros/slam_map');s=mapper.spec
    xmin,ymin,xmax,ymax=-4.5,-3.65,4.5,3.65
    # Sample only the published SLAM map; missing/unknown cells stay transparent.
    res=.025
    xx=xmin+(np.arange(round((xmax-xmin)/res))+.5)*res
    yy=ymin+(np.arange(round((ymax-ymin)/res))+.5)*res
    c=np.floor((xx-s.xmin)/s.resolution).astype(int)
    r=np.floor((yy-s.ymin)/s.resolution).astype(int)
    inside=(r[:,None]>=0)&(r[:,None]<s.height)&(c[None,:]>=0)&(c[None,:]<s.width)
    rr=np.clip(r,0,s.height-1);cc=np.clip(c,0,s.width-1)
    observed=mapper.observed[np.ix_(rr,cc)]&inside
    occupied=mapper.occupied()[np.ix_(rr,cc)]&inside
    rgba=np.zeros((*observed.shape,4),dtype=np.uint8)
    rgba[observed]=[245,248,249,255];rgba[occupied]=[35,63,84,255]
    fig=plt.figure(figsize=(10,10*(ymax-ymin)/(xmax-xmin)),dpi=240,facecolor='none')
    ax=fig.add_axes([0,0,1,1],facecolor='none')
    ax.imshow(rgba,origin='lower',extent=(xmin,xmax,ymin,ymax),interpolation='nearest')
    ax.plot(q[:,0],q[:,1],c='white',lw=3.6,solid_capstyle='round')
    ax.plot(q[:,0],q[:,1],c='#099a82',lw=1.8,solid_capstyle='round')
    ax.scatter(*q[0,:2],s=40,c='#287bd1',edgecolors='white',linewidths=.8,zorder=5)
    ax.scatter(*q[-1,:2],s=68,facecolors='none',edgecolors='#e0a240',marker='s',linewidths=1.6,zorder=6)
    ax.set(xlim=(xmin,xmax),ylim=(ymin,ymax));ax.set_aspect('equal');ax.set_axis_off()
    path=OUT/name
    fig.savefig(path,transparent=True,dpi=240,pad_inches=0);plt.close(fig)
    result=trim_alpha(path);result['source_run']=run.name
    return result

results.append(trajectory(RUN,'trajectory_best_new_scene_transparent.png'))
results.append(trajectory(NAV/'artifacts/autonomy_ros_open_passages_v2','trajectory_reference_loop_transparent.png'))
(OUT/'export_manifest.json').write_text(json.dumps({'assets':results,
    'pointcloud_source_run':RUN.name,'pointcloud_colors':'Native registered RGB, unchanged',
    'pointcloud_preview_cutaway':'South outer wall high points hidden, same as previous static preview',
    'trajectory_background':'Unknown map cells transparent; observed free cells and obstacles retained',
    'annotations':False,'axes':False,'titles':False,'metrics':False},indent=2),encoding='utf-8')
print(json.dumps(results,indent=2))
