"""Export actual 2D SLAM and native lidar returns accumulated using SLAM poses.

No source meshes or simulator truth poses are used to reconstruct the cloud.
Planar SLAM supplies x/y/yaw; fixed mounting height and level attitude are assumed.
"""
from pathlib import Path
import argparse,json,sys,base64
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
ROOT=Path(__file__).resolve().parents[1]
NAV=ROOT/'modules/wheeled_navigation'
sys.path.insert(0,str(NAV))
from scan_nav.core import OccupancyMapper
p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path)
p.add_argument('--rgb',type=Path,help='Native rendered RGB registered to these exact saved scan samples')
a=p.parse_args();RUN=a.run.resolve();OUT=RUN/'reconstruction';OUT.mkdir(exist_ok=True)
if a.rgb is None and (OUT/'native_rgb_colors.npz').exists():a.rgb=OUT/'native_rgb_colors.npz'
font='C:/Windows/Fonts/msyh.ttc';font_manager.fontManager.addfont(font)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),'axes.unicode_minus':False,
    'figure.facecolor':'#f2f6f9','text.color':'#19364c','font.size':11})
m=OccupancyMapper.load(RUN/'ros/slam_map');s=m.spec
extent=[s.xmin,s.xmin+s.width*s.resolution,s.ymin,s.ymin+s.height*s.resolution]
def draw_map(ax):
    data=np.zeros(m.observed.shape,dtype=np.uint8);data[m.free()]=1;data[m.occupied()]=2
    ax.imshow(data,origin='lower',extent=extent,cmap=ListedColormap(['#aab9c7','#f9fbfc','#203c51']),vmin=0,vmax=2)
    ax.set(xlim=(-4.6,4.6),ylim=(-3.8,3.8),xlabel='x / 米',ylabel='y / 米');ax.set_aspect('equal')
fig,ax=plt.subplots(figsize=(11,10));draw_map(ax)
ax.set_title('实际建图结果：SLAM Toolbox 二维占据栅格\n白色：已知空地  ·  深色：障碍  ·  灰色：未知',fontsize=17,pad=20)
fig.text(.5,.025,'单独显示地图，不叠加机器人轨迹，也不叠加源场景几何。',ha='center',fontsize=11)
fig.tight_layout(rect=(0,.05,1,1));fig.savefig(OUT/'slam_map_only.png',dpi=160);plt.close(fig)

cal=np.load(ROOT/'research/scene_audit/lidar_calibration/native_rays.npz')
directions=cal['directions']
est=np.asarray(json.loads((RUN/'ros/estimates.json').read_text()))
assert len(est)>2
with np.load(RUN/'sensor_history.npz') as h:
    assert np.allclose(h['angles'],cal['angles'],atol=1e-6)
    raw=h['raw_depth'];times=h['times'];fused=h['ranges'];angles=h['angles']
    assert raw.shape[1:]==directions.shape[:2],(raw.shape,directions.shape)
    # Check the recorded ray calibration against the exact virtual scan that
    # drove this experiment before using it to reconstruct any 3D surfaces.
    errors=[]
    for index in np.linspace(0,len(raw)-1,8,dtype=int):
        r=raw[index]*8./65535.;local=directions*r[...,None]
        z=.84+local[...,2]
        valid=(r>=.15)&(r<7.99)&(z>=.06)&(z<=1.12)
        scan=np.min(np.where(valid,np.linalg.norm(local[...,:2],axis=2),np.inf),axis=0)
        scan[~np.isfinite(scan)]=8.
        errors.append(float(np.max(np.abs(scan-fused[index]))))
    assert max(errors)<.002,errors
    indices=np.arange(0,len(raw),10)
    indices=indices[(times[indices]>=est[0,0])&(times[indices]<=est[-1,0])]
    ts=times[indices]
    poses=np.c_[np.interp(ts,est[:,0],est[:,1]),np.interp(ts,est[:,0],est[:,2]),
        np.interp(ts,est[:,0],np.unwrap(est[:,3]))]
    points=[];local_samples=[];source_frames=[]
    for index,pose in zip(indices,poses):
        ranges=raw[index,::2,::3]*8./65535.
        local=directions[::2,::3]*ranges[...,None]
        valid=(ranges>=.15)&(ranges<7.99)
        local=local[valid];sensor_local=local.copy();local[:,0]+=.16;local[:,2]+=.84
        cs,sn=np.cos(pose[2]),np.sin(pose[2])
        local[:,:2]=local[:,:2]@np.array([[cs,sn],[-sn,cs]])+pose[:2]
        keep=(local[:,2]>=-.03)&(local[:,2]<=3.2)
        local=local[keep]
        points.append(local)
        local_samples.append(sensor_local[keep]);source_frames.append(np.full(len(local),index,dtype=np.int32))
points=np.concatenate(points).astype(np.float32)
input_count=len(points);voxel=.035
_,unique=np.unique(np.floor(points/voxel).astype(np.int32),axis=0,return_index=True)
points=points[unique]
np.savez_compressed(OUT/'cloud_samples.npz',points=points,
    local_hits=np.concatenate(local_samples)[unique].astype(np.float32),
    source_frames=np.concatenate(source_frames)[unique])
color_label='原生 RGB 图像配色' if a.rgb else '尚未配色的灰色点云'
if a.rgb:
    with np.load(a.rgb) as rgb_data:
        assert np.array_equal(rgb_data['points'],points),'RGB colors belong to a different point cloud'
        colors=rgb_data['colors']
        assert colors.shape==points.shape
else:
    colors=np.full(points.shape,180,dtype=np.uint8)
record=np.empty(len(points),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1')])
for i,name in enumerate(('x','y','z')):record[name]=points[:,i]
for i,name in enumerate(('red','green','blue')):record[name]=colors[:,i]
header=('ply\nformat binary_little_endian 1.0\ncomment Actual Isaac lidar returns accumulated with 2D SLAM poses; not 6DoF 3D SLAM\n'
    f'element vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\n'
    'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
with (OUT/'slam_pose_cloud.ply').open('wb') as f:f.write(header.encode());record.tofile(f)
fig=plt.figure(figsize=(17,9));ax=fig.add_subplot(121,projection='3d')
# Cut away only the south outer wall for this illustration, keeping the
# complete point cloud in the PLY and the interactive viewer.
mask=~((points[:,1]<-3.5)&(points[:,2]>.3));display=np.flatnonzero(mask)[::2]
ax.scatter(*points[display].T,c=colors[display]/255,s=.6,depthshade=False,rasterized=True)
ax.set(xlim=(-4.7,4.7),ylim=(-3.9,3.9),zlim=(0,3),xlabel='x / 米',ylabel='y / 米',zlabel='高度 / 米')
ax.set_box_aspect((9,7.3,3));ax.view_init(elev=38,azim=-68)
if a.rgb:ax.set_facecolor('#bac5ce');ax.grid(False)
ax.set_title('三维雷达实测点云 + 二维 SLAM 位姿拼接\n'+color_label+'；静态图隐藏近侧外墙以便观察内部',fontsize=13,pad=15)
draw_map(fig.add_subplot(122));fig.axes[-1].set_title('同一轮实际二维 SLAM 地图\n没有叠加路线或源场景模型',fontsize=13,pad=15)
fig.suptitle('这次真正建出来的地图与点云',fontsize=24,fontweight='bold',y=.96)
fig.text(.5,.04,f'{len(points):,} 个点  ·  {voxel*100:.1f} 厘米体素  ·  {len(indices)} 帧实测扫描  ·  x / y / 朝向来自 SLAM，z / 横滚 / 俯仰采用平面运动假设',ha='center',fontsize=11)
fig.text(.5,.012,'这是三维扫描的累积展示，尚未运行完整六自由度三维 SLAM；未扫描到的表面保持缺失。',ha='center',fontsize=10,color='#62778a')
fig.subplots_adjust(left=.03,right=.97,bottom=.10,top=.85,wspace=.13)
fig.savefig(OUT/'reconstruction_overview.png',dpi=160);plt.close(fig)
if a.rgb:
    fig=plt.figure(figsize=(13,10),facecolor='#e5ebf0');ax=fig.add_subplot(111,projection='3d')
    ax.set_facecolor('#bac5ce')
    ax.scatter(*points[display].T,c=colors[display]/255,s=1.,depthshade=False,rasterized=True)
    ax.set(xlim=(-4.7,4.7),ylim=(-3.9,3.9),zlim=(0,3),xlabel='x / 米',ylabel='y / 米',zlabel='高度 / 米')
    ax.set_box_aspect((9,7.3,3));ax.view_init(elev=42,azim=-68);ax.grid(False)
    fig.suptitle('原场景颜色 · 激光点云',fontsize=25,fontweight='bold',y=.95)
    fig.text(.5,.9,'白色墙面、木色地板与家具、浅色床品，来自同一 Isaac 场景的 RGB 渲染',ha='center',fontsize=12)
    fig.text(.5,.055,'实际雷达回波 + SLAM 平面位姿拼接；只给已扫描点配色，没有补出未知表面。',ha='center',fontsize=11)
    fig.text(.5,.025,'静态预览隐藏近侧外墙高处点；PLY 和交互文件保留全部回波。',ha='center',fontsize=10,color='#62778a')
    fig.subplots_adjust(left=.01,right=.99,bottom=.09,top=.86)
    fig.savefig(OUT/'rgb_pointcloud.png',dpi=170);plt.close(fig)

positions=base64.b64encode(points.astype('<f4').tobytes()).decode()
rgb=base64.b64encode((colors.astype(np.float32)/255).astype('<f4').tobytes()).decode()
html='''<!doctype html><html lang="zh"><meta charset="utf-8"><title>实际雷达点云</title>
<style>body{margin:0;background:#102336;color:#eaf3fb;font:15px 'Microsoft YaHei',sans-serif}header{padding:18px 24px}h2{margin:0 0 8px}p{margin:5px 0;color:#bed1df}canvas{display:block;width:100vw;height:calc(100vh - 160px)}button{margin:8px 8px 0 0;padding:7px 16px;cursor:pointer;border:0;border-radius:5px}</style>
<header><h2>实际雷达点云 · SLAM 位姿拼接</h2><p>__COUNT__ 个点。拖动旋转，滚轮缩放。__COLOR__</p><p>二维 SLAM 提供 x / y / 朝向；固定传感器高度、横滚和俯仰。尚未运行完整六自由度三维 SLAM。</p><button id="reset">重置视角</button><button id="top">俯视</button></header><canvas id="c"></canvas><script>
const c=document.getElementById('c'),gl=c.getContext('webgl',{antialias:true});
if(!gl)throw Error('此浏览器不支持 WebGL');
function floats(b){const s=atob(b),a=new Uint8Array(s.length);for(let i=0;i<s.length;i++)a[i]=s.charCodeAt(i);return new Float32Array(a.buffer)}
const pos=floats('__POS__'),rgb=floats('__RGB__');
function shader(t,s){const q=gl.createShader(t);gl.shaderSource(q,s);gl.compileShader(q);if(!gl.getShaderParameter(q,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(q));return q}
const program=gl.createProgram();gl.attachShader(program,shader(gl.VERTEX_SHADER,`attribute vec3 p;attribute vec3 col;varying vec3 v;uniform float a,b,scale,aspect,size;void main(){vec3 q=p-vec3(0.,0.,1.2);float x=cos(a)*q.x-sin(a)*q.y;float h=sin(a)*q.x+cos(a)*q.y;float y=sin(b)*q.z-cos(b)*h;float d=sin(b)*h+cos(b)*q.z;gl_Position=vec4(x/(scale*aspect),y/scale,-d/30.,1.);gl_PointSize=size;v=col;}`));
gl.attachShader(program,shader(gl.FRAGMENT_SHADER,'precision mediump float;varying vec3 v;void main(){gl_FragColor=vec4(v,1.);}'));gl.linkProgram(program);gl.useProgram(program);
for(const [name,data] of [['p',pos],['col',rgb]]){const buf=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buf);gl.bufferData(gl.ARRAY_BUFFER,data,gl.STATIC_DRAW);const loc=gl.getAttribLocation(program,name);gl.enableVertexAttribArray(loc);gl.vertexAttribPointer(loc,3,gl.FLOAT,false,0,0)}
let a=-.5,b=.8,scale=5.5,drag=false,last;
function draw(){const dpr=window.devicePixelRatio||1;c.width=c.clientWidth*dpr;c.height=c.clientHeight*dpr;gl.viewport(0,0,c.width,c.height);gl.clearColor(.055,.10,.16,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.enable(gl.DEPTH_TEST);for(const [k,v] of Object.entries({a,b,scale,aspect:c.width/c.height,size:2*dpr}))gl.uniform1f(gl.getUniformLocation(program,k),v);gl.drawArrays(gl.POINTS,0,pos.length/3)}
c.onpointerdown=e=>{drag=true;last=[e.clientX,e.clientY];c.setPointerCapture(e.pointerId)};c.onpointerup=()=>drag=false;c.onpointermove=e=>{if(!drag)return;a+=(e.clientX-last[0])*.008;b=Math.max(.02,Math.min(3.12,b+(e.clientY-last[1])*.008));last=[e.clientX,e.clientY];draw()};c.onwheel=e=>{e.preventDefault();scale=Math.max(.8,Math.min(20,scale*Math.exp(e.deltaY*.001)));draw()};document.getElementById('reset').onclick=()=>{a=-.5;b=.8;scale=5.5;draw()};document.getElementById('top').onclick=()=>{a=0;b=.001;scale=5.5;draw()};window.onresize=draw;draw();</script></html>'''
html=html.replace('__COUNT__',f'{len(points):,}').replace('__POS__',positions).replace('__RGB__',rgb)
html=html.replace('__COLOR__','颜色来自同一场景的原生 RGB 渲染，已经配准到实际雷达回波。' if a.rgb else '尚未配色的点以灰色显示。')
(OUT/'pointcloud_viewer.html').write_text(html,encoding='utf-8')
report={'run':str(RUN),'points':len(points),'input_return_count':input_count,'sampled_frames':len(indices),
    'voxel_m':voxel,'pose_source':'Saved SLAM Toolbox map-to-base TF estimates, time-interpolated; simulator truth is not read.',
    'sensor_source':'Recorded native PhysX uint16 3D lidar ranges; native API ray calibration with matching angular layout.',
    'ray_calibration_fused_scan_max_errors_m':errors,'max_calibration_scan_error_m':max(errors),
    'mount_xyz_m':[.16,0,.84],'attitude_assumption':'Fixed z, roll=0, pitch=0; SLAM estimates x,y,yaw only.',
    'full_6dof_3d_slam':False,'source_scene_geometry_used':False,
    'point_color':'Registered native RGB camera rendering' if a.rgb else 'Uncolored neutral gray',
    'rgb_source':str(a.rgb.resolve()) if a.rgb else None,
    'static_plot_only_cutaway':'South outer wall points y<-3.5,z>0.3 hidden in static plot only; PLY/HTML retain all returns.'}
(OUT/'reconstruction_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
