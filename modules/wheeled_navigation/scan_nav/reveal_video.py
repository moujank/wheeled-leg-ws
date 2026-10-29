"""Presentation helpers for a measured-scan reveal, not photometric lighting.

XY visibility lights floor/wall columns; a hit reveals the corresponding mesh
part. This is an explanatory visualization, not reconstructed 3D surfaces.
"""
from pathlib import Path
import math
import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT='C:/Windows/Fonts/msyh.ttc'
BOLD='C:/Windows/Fonts/msyhbd.ttc'

def font(size,bold=False):
    return ImageFont.truetype(BOLD if bold else FONT,size)

def route_motion(route,fps,tour_seconds,intro_seconds,outro_seconds):
    """A planar world-frame velocity command, integrated into the model root.

    Acceleration ramps at start/end; heading rate is capped. This is a
    holonomic visualization carrier, not a two-wheel robot controller.
    """
    route=np.asarray(route,dtype=float)
    lengths=np.r_[0,np.cumsum(np.linalg.norm(np.diff(route,axis=0),axis=1))]
    lengths,indices=np.unique(lengths,return_index=True);route=route[indices]
    duration=intro_seconds+tour_seconds+outro_seconds
    count=round(duration*fps)
    def position(distance):
        return np.array([np.interp(distance,lengths,route[:,0]),np.interp(distance,lengths,route[:,1])])
    first=position(0);ahead=position(.12)
    yaw=math.atan2(*(ahead-first)[::-1]);previous=np.r_[first,yaw]
    for frame in range(count):
        t=frame/fps;u=np.clip((t-intro_seconds)/tour_seconds,0,1)
        # Sinusoidal speed envelope: exact start/end, mostly steady mid-tour.
        distance=lengths[-1]*(u-math.sin(2*math.pi*u)/(2*math.pi))
        point=position(distance)
        a=position(min(lengths[-1],distance+.16));delta=a-point
        desired=math.atan2(delta[1],delta[0]) if np.linalg.norm(delta)>1e-5 else yaw
        error=math.atan2(math.sin(desired-yaw),math.cos(desired-yaw))
        wz=np.clip(error*7,-2.1,2.1) if t>=intro_seconds else 0
        yaw+=wz/fps
        pose=np.r_[point,yaw]
        command=np.r_[(pose[:2]-previous[:2])*fps,(pose[2]-previous[2])*fps]
        # Explicit integration is the only root motion used by the renderer.
        pose=previous+command/fps
        yield t,pose,command,float(distance/lengths[-1])
        previous=pose

class RevealComposer:
    def __init__(self,width,height,render_width,spec,world_xy,scene_mask,part_ids,part_count):
        self.width,self.height,self.render_width=width,height,render_width
        self.spec=spec;self.scene_mask=scene_mask
        self.part_ids=part_ids
        self.scan_layers=17
        self.first=np.full((spec.height,spec.width),np.inf,dtype=np.float32)
        self.part_first=np.full(part_count+1,np.inf,dtype=np.float32)
        self.cols=np.floor((world_xy[...,0]-spec.xmin)/spec.resolution).astype(np.int32)
        self.rows=np.floor((world_xy[...,1]-spec.ymin)/spec.resolution).astype(np.int32)
        self.inside=(self.cols>=0)&(self.cols<spec.width)&(self.rows>=0)&(self.rows<spec.height)&scene_mask
        self.cols=np.clip(self.cols,0,spec.width-1);self.rows=np.clip(self.rows,0,spec.height-1)
        self.layout=Image.new('RGB',(width,height),'#0b1722')
        d=ImageDraw.Draw(self.layout)
        x=render_width+28
        d.rounded_rectangle((render_width+9,20,width-18,height-20),radius=20,fill='#112330',outline='#23414e',width=1)
        d.text((x,44),'CODE-AS-ROOM',font=font(27,True),fill='#64ddc7')
        d.text((x,92),'雷达探索',font=font(40,True),fill='#f0f5f7')
        d.text((x,151),'ISAAC SIM  /  真实测距',font=font(19),fill='#9db4c3')
        d.line((x,195,width-42,195),fill='#294552',width=1)
        d.text((x,228),'累计观测范围',font=font(23,True),fill='#eef5f7')
        d.text((x,565),'蓝线  预设路线',font=font(20),fill='#628fc5')
        d.text((x,603),'青色  当前扫描与已走轨迹',font=font(19),fill='#66dec4')
        d.text((x,641),'灰蓝  尚未观测',font=font(20),fill='#8d9daa')
        d.line((x,865,width-42,865),fill='#294552',width=1)
        d.text((x,890),'亮度表示观测进度',font=font(21,True),fill='#e6f1f4')
        d.text((x,932),'地面可见范围 / 命中家具部件',font=font(17),fill='#91aab9')
        d.text((x,971),'预设运动 · 仿真真值位姿',font=font(18),fill='#91aab9')
        d.text((x,1008),'不调用 RL 策略',font=font(18),fill='#91aab9')
        self.phase_names=['等待开始','沿预设路线扫描','扫描完成 · 保留遮挡区域']

    def observe(self,seen,hit_parts,time):
        new=seen&~np.isfinite(self.first)
        self.first[new]=time
        for part in hit_parts:
            if part>0 and not np.isfinite(self.part_first[part]):self.part_first[part]=time

    def compose(self,rgb,robot_mask,scan_mask,time,pose,progress,route_xy,phase,scan_count):
        # First-observation timestamps make reveal permanent, with a short fade.
        from scipy.ndimage import gaussian_filter
        field=gaussian_filter(np.clip((time-self.first)/.36,0,1),sigma=.85)
        xy_reveal=field[self.rows,self.cols];xy_reveal[~self.inside]=0
        part_reveal=np.clip((time-self.part_first[self.part_ids])/.5,0,1)
        reveal=np.where(self.part_ids>0,part_reveal,xy_reveal)
        source=rgb.astype(np.float32)
        luma=source.mean(axis=2,keepdims=True)
        dark=source*.09+luma*.045+np.array([4,10,17],np.float32)
        output=dark+(source-dark)*reveal[...,None]
        output[~self.scene_mask]=[11,23,34]
        # Robot and visible laser lines are readable even in unexplored space.
        output[robot_mask]=source[robot_mask]*.95+np.array([0,4,5])
        output[scan_mask]=np.minimum(255,source[scan_mask]*1.15+np.array([0,24,18]))
        canvas=self.layout.copy()
        canvas.paste(Image.fromarray(np.uint8(np.clip(output,0,255))),(0,0))
        d=ImageDraw.Draw(canvas);x=self.render_width+28
        d.rounded_rectangle((30,28,self.render_width-30,113),radius=17,fill='#10232e')
        d.text((54,42),'让房间随着扫描逐渐显现',font=font(29,True),fill='#edf6f6')
        d.text((56,82),'Code-as-Room  ·  保留两道隔墙  ·  前侧外墙剖视',font=font(17),fill='#99b7c6')
        d.rounded_rectangle((36,self.height-86,self.render_width-36,self.height-27),radius=12,fill='#10232e')
        d.ellipse((57,self.height-64,70,self.height-51),fill='#66dec4')
        d.text((84,self.height-75),self.phase_names[phase],font=font(22),fill='#e1f2f3')
        d.text((self.render_width-240,self.height-73),f'{time:04.1f} s',font=font(21),fill='#91b5c4')
        # Small evidence map uses the same accumulated visibility, not scene boxes.
        mini=np.full((*self.first.shape,3),[33,51,66],dtype=np.uint8)
        mini[np.isfinite(self.first)]=[136,186,181]
        preview=Image.fromarray(np.flipud(mini)).resize((300,208),Image.Resampling.NEAREST)
        canvas.paste(preview,(x,285));d=ImageDraw.Draw(canvas)
        def to_mini(points):
            p=np.asarray(points)
            return np.c_[x+(p[:,0]-self.spec.xmin)/(self.spec.width*self.spec.resolution)*300,
                         285+208-(p[:,1]-self.spec.ymin)/(self.spec.height*self.spec.resolution)*208]
        route=to_mini(route_xy)
        d.line([tuple(p) for p in route],fill='#557ba2',width=1)
        px,py=to_mini([pose[:2]])[0]
        d.ellipse((px-5,py-5,px+5,py+5),fill='#83ffe0')
        d.line((px,py,px+13*math.cos(pose[2]),py-13*math.sin(pose[2])),fill='#d0fff3',width=2)
        percent=100*np.isfinite(self.first[16:256,16:376]).mean()
        d.text((x,515),f'地面投影覆盖  {percent:04.1f}%',font=font(22,True),fill='#79e0c7')
        d.text((x,707),f'路线进度  {progress*100:05.1f}%',font=font(23,True),fill='#eff7f8')
        d.rounded_rectangle((x,752,self.width-45,763),radius=5,fill='#294452')
        if progress>0:d.rounded_rectangle((x,752,x+(self.width-45-x)*progress,763),radius=5,fill='#5fd6be')
        d.text((x,796),f'扫描帧  {scan_count:04d}   /   {self.scan_layers} 线',font=font(20),fill='#a5bdca')
        return canvas
