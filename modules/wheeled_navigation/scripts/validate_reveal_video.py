"""Check encoded video, monotone scan reveal, and integrated planar commands."""
import argparse
import json
import math
import subprocess
import sys
from pathlib import Path
import numpy as np
from PIL import Image

MODULE=Path(__file__).resolve().parents[1]
WORKSPACE=MODULE.parents[1]
sys.path.insert(0,str(WORKSPACE/'research/video_deps'))
import imageio_ffmpeg

p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path)
a=p.parse_args();out=a.output
meta=json.loads((out/'render_status.json').read_text(encoding='utf-8'))
assert meta['status']=='passed',meta
motion=json.loads((out/'motion_commands.json').read_text(encoding='utf-8'))
scans=np.load(out/'animation_scans.npz',allow_pickle=False)
assert len(motion)==meta['frames']
poses=np.array([v['pose'] for v in motion]);commands=np.array([v['command_world_vx_vy_wz'] for v in motion])
np.testing.assert_allclose(np.diff(poses,axis=0),commands[1:]/meta['fps'],atol=1e-12,rtol=0)
assert np.linalg.norm(poses[0,:2]-np.array([.9,-1.4]))<1e-9
assert np.linalg.norm(poses[-1,:2]-poses[0,:2])<1e-9,'Preset tour did not return to its start'
assert np.max(np.linalg.norm(np.diff(poses[:,:2],axis=0),axis=1))<.08,'Discontinuous root translation'
assert np.max(abs(commands[:,2]))<=2.100001,'Discontinuous heading'
assert scans['raw_depth_uint16'].dtype==np.uint16
assert scans['raw_depth_uint16'].shape==(meta['scan_frames'],meta['vertical_channels'],360)
assert scans['local_hit_points'].shape==(*scans['raw_depth_uint16'].shape,3)
assert np.all(np.diff(scans['xy_coverage_fraction'])>=0)
assert np.all(np.diff(scans['hit_part_count'])>=0)
assert (np.isinf(scans['first_observed_time'])).any(),'All space was artificially revealed'
pose_data=json.loads((WORKSPACE/'research/scene_audit/code_as_room/wheeled_arm_v2_bent_visual.json').read_text())
assert pose_data['joint_angles_rad']['HipL']==.65
assert pose_data['joint_angles_rad']['KneeL']==1.3
assert .0<=pose_data['wheel_min_z_m']<.01

video=out/'code_as_room_lidar_reveal.mp4'
assert video.stat().st_size>100_000
reader=imageio_ffmpeg.read_frames(str(video),pix_fmt='rgb24')
encoded=next(reader)
assert list(encoded['size'])==meta['resolution'],encoded
assert abs(encoded['fps']-meta['fps'])<.001
assert abs(encoded['duration']-meta['duration_s'])<.1
count=0;first=None;last=None
for raw in reader:
    rgb=np.frombuffer(raw,dtype=np.uint8).reshape(meta['resolution'][1],meta['resolution'][0],3)
    if first is None:first=rgb.copy()
    last=rgb.copy();count+=1
assert count==meta['frames'],(count,meta['frames'])
scene=np.load(out/'camera_surface_projection.npz',allow_pickle=False)['scene_mask']
initial=float(first[:,:1536][scene].mean())
final=float(last[:,:1536][scene].mean())
assert final>initial*1.8,(initial,final)
Image.fromarray(first).save(out/'encoded_first_frame.png')
Image.fromarray(last).save(out/'encoded_final_frame.png')
with Image.open(out/'code_as_room_lidar_reveal.gif') as gif:
    assert gif.n_frames>50
    assert gif.size==(960,540)
    gif_frames=gif.n_frames
result=dict(status='passed',decoded_mp4_frames=count,gif_frames=gif_frames,
    resolution=encoded['size'],fps=encoded['fps'],duration_s=encoded['duration'],
    initial_scene_mean_brightness=initial,final_scene_mean_brightness=final,
    reveal_monotone=True,velocity_commands_reproduce_root_motion=True,
    robot='wheeled_arm_v2; HipL/R=+/-0.65, KneeL/R=+/-1.3 rad',
    wheel_bottom_z_m=pose_data['wheel_min_z_m'])
(out/'media_validation.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result))
