"""Frontier coordinator for REAL SLAM Toolbox and Nav2 actions.

Run in the project Pixi ROS environment (not Isaac's Python). It subscribes to
/map and TF, sends NavigateToPose, and records results. It cannot open a USD,
read simulator truth, or obtain the room's mesh/collider layout.
"""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid,Path as RosPath,Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer,TransformListener

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scan_nav.exploration import FrontierExplorer,from_occupancy
from scan_nav.footprint import INSCRIBED_RADIUS,feasible_yaws


class ExplorerNode(Node):
    def __init__(self,out,limit):
        super().__init__('wheeled_frontier_explorer',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.out=out;self.limit=limit;self.mapper=None;self.map_count=0
        self.explorer=FrontierExplorer(radius=INSCRIBED_RADIUS,polygon=True);self.phase='waiting_for_slam_nav2';self.active=None
        self.goal_handle=None;self.pending=False;self.goal_started=0.;self.start=None;self.empty=0
        self.active_frontier=None
        self.events=[];self.estimates=[];self.paths=[];self.path=[];self.commands=[];self.odometry=[]
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.client=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.compute=ActionClient(self,ComputePathToPose,'compute_path_to_pose')
        self.cost_mapper=None;self.costmap_messages={};self.probe_options=[];self.probe_target=None;self.probe_original=None
        self.state_client=self.create_client(GetState,'/bt_navigator/get_state')
        self.nav_active=False;self.state_pending=False
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,latched)
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.on_costmap,latched)
        self.create_subscription(OccupancyGrid,'/local_costmap/costmap',lambda msg:self.costmap_messages.update(local=msg),latched)
        self.create_subscription(RosPath,'/plan',self.on_path,10)
        self.create_subscription(Twist,'/cmd_vel',self.on_command,10)
        self.create_subscription(Odometry,'/odom',self.on_odom,10)
        self.status_pub=self.create_publisher(String,'/exploration/status',10)
        self.timer=self.create_timer(1.,self.tick)
        self.started=None;self.latest_pose=None;self.first_map_cells=None
        self.best_area=0.;self.last_gain_time=0.;self.return_requested=False
        self.bootstrap_until=0.
        self.bootstrap_pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.bootstrap_timer=self.create_timer(.1,self.bootstrap_command)

    def seconds(self):return self.get_clock().now().nanoseconds/1e9

    def event(self,kind,**kw):
        row={'event':kind,'seconds':self.seconds(),**kw};self.events.append(row)
        print('ROS_EXPLORATION '+json.dumps(row),flush=True)

    def on_map(self,msg):
        grid=np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
        self.mapper=from_occupancy(grid,msg.info.resolution,(msg.info.origin.position.x,msg.info.origin.position.y))
        self.map_count+=1
        area=float((grid>=0).sum())*msg.info.resolution**2
        if area>self.best_area+.15:
            self.best_area=area;self.last_gain_time=self.seconds()
        if self.first_map_cells is None:self.first_map_cells=int((grid>=0).sum())

    def on_path(self,msg):
        self.path=[(q.pose.position.x,q.pose.position.y) for q in msg.poses]
        self.paths.append({'seconds':self.seconds(),'path':self.path})

    def on_costmap(self,msg):
        self.costmap_messages['global']=msg
        grid=np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
        # OccupancyGrid published by Nav2: 100 is lethal, 99 is inscribed,
        # 1..98 are soft inflation costs. Do not inflate those a second time.
        raw=np.where(grid<0,-1,np.where(grid==100,100,0))
        self.cost_mapper=from_occupancy(raw,msg.info.resolution,(msg.info.origin.position.x,msg.info.origin.position.y))

    def on_command(self,msg):
        self.commands.append((self.seconds(),float(msg.linear.x),float(msg.angular.z)))

    def on_odom(self,msg):
        q=msg.pose.pose
        self.odometry.append((self.seconds(),q.position.x,q.position.y,
                              2*math.atan2(q.orientation.z,q.orientation.w)))

    def tick(self):
        now=self.seconds()
        if not self.nav_active and not self.state_pending and self.state_client.service_is_ready():
            self.state_pending=True
            self.state_client.call_async(GetState.Request()).add_done_callback(self.on_state)
        if not self.started and now>0:self.started=now
        try:
            t=self.buffer.lookup_transform('map','base_footprint',Time()).transform
            pose=(t.translation.x,t.translation.y,2*math.atan2(t.rotation.z,t.rotation.w))
        except Exception:
            self.publish();return
        self.latest_pose=pose;self.estimates.append((now,*pose))
        if self.mapper is None or not self.client.server_is_ready() or not self.nav_active:self.publish();return
        if self.start is None:
            # This sensor already measures 360 degrees in every frame. Keep the
            # initial pose while the first map settles; a forced turn gives no
            # extra azimuth coverage and needlessly changes approach heading.
            self.start=pose;self.phase='initial_scan';self.bootstrap_until=now+2.
            self.last_gain_time=now
            self.event('stack_ready',first_map_known_cells=self.first_map_cells,
                       simulator_truth_subscribed=False,planner='Nav2 Smac Hybrid A* / polygon SE(2)',
                       controller='Nav2 '+os.environ.get('WHEELED_NAV_CONTROLLER','dwb'))
        if self.phase=='initial_scan' and now>=self.bootstrap_until:
            self.bootstrap_pub.publish(Twist());self.phase='exploring'
            self.event('initial_scan_finished',new_known_cells=int(self.mapper.observed.sum()))
        if self.phase=='initial_scan':self.publish();return
        if self.active and now-self.goal_started>60 and self.goal_handle:
            self.goal_handle.cancel_goal_async();self.event('goal_cancel_requested',goal=self.active,reason='timeout')
            self.goal_started=now+1000
        if self.return_requested and self.active is None and not self.pending:
            self.return_requested=False;self.phase='returning';self.send(self.start[:2],pose,'return_to_start')
        reached=sum(e['event']=='goal_result' and e['success'] for e in self.events)
        if self.phase=='exploring' and not self.return_requested and reached>=3 and now-self.last_gain_time>50:
            self.event('exploration_stagnation',seconds_without_new_area=now-self.last_gain_time,
                       reason='No gain above 0.15 square metre for 50 simulation seconds; not full-scene coverage')
            self.return_requested=True
            if self.goal_handle:self.goal_handle.cancel_goal_async()
        if self.phase=='exploring' and not self.return_requested and not self.pending and self.active is None:
            goal=self.explorer.choose(self.cost_mapper or self.mapper,pose,now)
            if goal:
                self.empty=0;self.prepare_goal(goal,pose)
            else:
                self.empty+=1
                if self.empty>=4:
                    self.event('no_reachable_frontier',diagnostics=self.explorer.last_diagnostics)
                    self.phase='returning';self.send(self.start[:2],pose,'return_to_start')
        if self.started and now-self.started>self.limit and self.phase=='exploring' and not self.return_requested:
            self.event('exploration_time_budget_reached')
            if self.goal_handle:self.goal_handle.cancel_goal_async()
            self.return_requested=True
        self.publish()
        if len(self.estimates)%10==0:self.save()

    def send(self,xy,pose,purpose,yaw=None,frontier=None):
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
        goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.pose.pose.position.x=float(xy[0]);goal.pose.pose.position.y=float(xy[1])
        if yaw is None:yaw=self.start[2] if purpose=='return_to_start' else math.atan2(xy[1]-pose[1],xy[0]-pose[0])
        goal.pose.pose.orientation.z=math.sin(yaw/2);goal.pose.pose.orientation.w=math.cos(yaw/2)
        self.active=tuple(xy);self.active_frontier=frontier;self.pending=True;self.goal_started=self.seconds()
        self.event('goal_sent',goal=self.active,yaw=yaw,purpose=purpose,map_updates=self.map_count,
                   frontier_diagnostics=self.explorer.last_diagnostics)
        future=self.client.send_goal_async(goal)
        future.add_done_callback(self.accepted)

    def prepare_goal(self,goal,pose):
        """Advance toward distant frontiers through currently measured space.

        These intermediate targets come from a newly computed map path, never a
        scene itinerary. A short advance fills missing viewpoints before asking
        a full-body SE(2) planner to traverse a still poorly observed passage.
        """
        mapper=self.cost_mapper or self.mapper
        target=goal.position;preferred=goal.yaw or pose[2];purpose='frontier'
        if goal.travel_distance>1.5:
            travelled=0.;choices=[]
            for index,(aa,bb) in enumerate(zip(goal.path,goal.path[1:])):
                travelled+=math.dist(aa,bb)
                if .65<=travelled<=1.25:
                    after=goal.path[min(index+12,len(goal.path)-1)]
                    before=goal.path[max(0,index-12)]
                    heading=math.atan2(after[1]-before[1],after[0]-before[0])
                    # A centre cell close to a doorway edge can allow only a
                    # sideways final pose, even though no turn is possible
                    # inside the passage. Search nearby measured cells for a
                    # pose aligned with the incoming path instead.
                    normal=np.array([-math.sin(heading),math.cos(heading)])
                    for offset in (0.,-.025,.025,-.05,.05):
                        xy=tuple(np.asarray(bb)+normal*offset)
                        headings=feasible_yaws(mapper,xy,heading)
                        for candidate_yaw in headings[:8]:
                            error=abs(math.atan2(math.sin(candidate_yaw-heading),math.cos(candidate_yaw-heading)))
                            alignment=min(error,abs(math.pi-error))
                            if alignment<.4:
                                choices.append((travelled-.7*alignment-.4*abs(offset),xy,candidate_yaw))
                                break
            if choices:
                _,target,preferred=max(choices,key=lambda q:q[0]);purpose='frontier_approach'
        preferred=min((preferred,preferred+math.pi),key=lambda yaw:
                      abs(math.atan2(math.sin(yaw-pose[2]),math.cos(yaw-pose[2]))))
        headings=feasible_yaws(mapper,target,preferred)
        options=[]
        for heading in headings:
            if all(abs(math.atan2(math.sin(heading-old),math.cos(heading-old)))>.65 for old in options):
                options.append(heading)
            if len(options)>=4:break
        if any(not success and self.seconds()-when<90 and math.dist(target,old)<.4
               for old,success,when in self.explorer.attempts):
            self.explorer.record(goal.position,False,self.seconds())
            self.event('approach_recently_failed',frontier=goal.position,target=target)
            return
        self.probe_target=tuple(target);self.probe_original=goal.position
        self.probe_purpose=purpose;self.probe_options=options;self.pending=True
        self.event('viewpoint_proposed',frontier=goal.position,target=target,purpose=purpose,yaw_options=options)
        self.try_probe()

    def try_probe(self):
        if self.return_requested:
            self.pending=False;return
        if not self.probe_options:
            self.event('viewpoint_no_se2_path',goal=self.probe_target,frontier=self.probe_original)
            self.explorer.record(self.probe_target,False,self.seconds());self.pending=False
            if math.dist(self.probe_target,self.probe_original)>.1:
                self.explorer.record(self.probe_original,False,self.seconds())
            return
        yaw=self.probe_options.pop(0);self.probe_yaw=yaw
        request=ComputePathToPose.Goal();request.goal.header.frame_id='map'
        request.goal.header.stamp=self.get_clock().now().to_msg()
        request.goal.pose.position.x=float(self.probe_target[0]);request.goal.pose.position.y=float(self.probe_target[1])
        request.goal.pose.orientation.z=math.sin(yaw/2);request.goal.pose.orientation.w=math.cos(yaw/2)
        request.planner_id='GridBased';request.use_start=False
        self.compute.send_goal_async(request).add_done_callback(self.probe_accepted)

    def probe_accepted(self,future):
        handle=future.result()
        if not handle.accepted:self.try_probe();return
        handle.get_result_async().add_done_callback(self.probe_finished)

    def probe_finished(self,future):
        result=future.result()
        if result.status==4 and result.result.path.poses:
            self.pending=False
            if not self.return_requested:
                self.send(self.probe_target,self.latest_pose,self.probe_purpose,yaw=self.probe_yaw,
                          frontier=self.probe_original)
        else:
            self.event('heading_path_rejected',goal=self.probe_target,yaw=self.probe_yaw)
            self.try_probe()

    def on_state(self,future):
        self.state_pending=False
        self.nav_active=future.result().current_state.id==3

    def bootstrap_command(self):
        if self.phase=='initial_scan':
            self.bootstrap_pub.publish(Twist())

    def accepted(self,future):
        self.pending=False;self.goal_handle=future.result()
        if not self.goal_handle.accepted:
            self.event('goal_rejected',goal=self.active)
            self.explorer.record(self.active,False,self.seconds());self.active=None;self.goal_handle=None
            if self.phase=='returning':self.phase='return_failed'
            return
        if self.return_requested:self.goal_handle.cancel_goal_async()
        self.goal_handle.get_result_async().add_done_callback(self.finished)

    def finished(self,future):
        status=int(future.result().status);success=status==4
        self.event('goal_result',goal=self.active,status=status,success=success,phase=self.phase)
        if self.active:self.explorer.record(self.active,success,self.seconds())
        if not success and self.active_frontier and math.dist(self.active,self.active_frontier)>.1:
            self.explorer.record(self.active_frontier,False,self.seconds())
        if self.phase=='returning':self.phase='complete' if success else 'return_failed'
        self.active=None;self.goal_handle=None;self.active_frontier=None

    def publish(self):
        payload={'phase':self.phase,'goal':self.active,'path':self.path,'map_updates':self.map_count,
                 'goals_sent':sum(e['event']=='goal_sent' for e in self.events),
                 'goals_reached':sum(e['event']=='goal_result' and e['success'] for e in self.events)}
        self.status_pub.publish(String(data=json.dumps(payload)))
        (self.out/'progress.json').write_text(json.dumps(payload,indent=2),encoding='utf-8')

    def save(self):
        self.out.mkdir(parents=True,exist_ok=True)
        if self.mapper:self.mapper.save(self.out/'slam_map')
        for name,msg in self.costmap_messages.items():
            try:
                t=self.buffer.lookup_transform(msg.header.frame_id,'base_footprint',Time()).transform
                pose=(t.translation.x,t.translation.y,2*math.atan2(t.rotation.z,t.rotation.w))
            except Exception:pose=(float('nan'),)*3
            np.savez_compressed(self.out/(name+'_costmap.npz'),
                data=np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width),
                resolution=msg.info.resolution,origin=(msg.info.origin.position.x,msg.info.origin.position.y),
                frame=msg.header.frame_id,pose=pose,seconds=self.seconds())
        for name,value in [('events',self.events),('estimates',self.estimates),('plans',self.paths),
                           ('commands',self.commands),('odometry',self.odometry)]:
            (self.out/(name+'.json')).write_text(json.dumps(value),encoding='utf-8')
        report={'phase':self.phase,'map_updates':self.map_count,'path_messages':len(self.paths),
                'command_messages':len(self.commands),'nonzero_commands':sum(abs(c[1])+abs(c[2])>1e-4 for c in self.commands),
                'goals_sent':sum(e['event']=='goal_sent' for e in self.events),
                'goals_reached':sum(e['event']=='goal_result' and e['success'] for e in self.events),
                'first_map_known_cells':self.first_map_cells,
                'last_map_known_cells':int(self.mapper.observed.sum()) if self.mapper else 0,
                'real_slam_toolbox':True,'real_nav2':True,'truth_subscribed':False}
        (self.out/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--limit',type=float,default=200.);p.add_argument('--wall-limit',type=float,default=1200.)
    p.add_argument('--launch-stack',action='store_true')
    p.add_argument('--sim-status-file',type=Path)
    p.add_argument('--domain-id',type=int,default=42)
    a=p.parse_args()
    a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=True)
    process=None;log=None
    os.environ['ROS_DOMAIN_ID']=str(a.domain_id)
    rclpy.init(domain_id=a.domain_id);node=ExplorerNode(a.output,a.limit);begin=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-begin<a.wall_limit:
            rclpy.spin_once(node,timeout_sec=.1)
            if a.sim_status_file and a.sim_status_file.is_file():
                try:
                    if json.loads(a.sim_status_file.read_text()).get('execution_finished'):break
                except (ValueError,OSError):pass
            if a.launch_stack and process is None and node.odometry:
                # Bring up lifecycle nodes after the live clock/TF publishers.
                log=(a.output/'stack.log').open('w',encoding='utf-8')
                process=subprocess.Popen([sys.executable,'-c','from ros2cli.cli import main; main()',
                    'launch',str(ROOT/'launch/slam_nav.launch.py')],stdout=log,stderr=subprocess.STDOUT)
            if node.phase in ('complete','return_failed','budget_reached'):
                node.publish();node.save()
                # Give the simulator enough status deliveries to stop gracefully.
                for _ in range(15):rclpy.spin_once(node,timeout_sec=.1)
                break
    finally:
        node.save();node.destroy_node();rclpy.shutdown()
        if process:
            subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True)
        if log:log.close()

if __name__=='__main__':main()
