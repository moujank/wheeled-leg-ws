"""One command starts the isolated ROS stack and Isaac simulator, then cleans up."""
from __future__ import annotations
import argparse
from datetime import datetime
import json
import os
import secrets
import shutil
from pathlib import Path
import subprocess
import sys
import time

MODULES=Path(__file__).resolve().parents[2]
WORKSPACE=MODULES.parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage',choices=['native','ros2'],default='ros2')
    p.add_argument('--scene',choices=['code-as-room','validation'],default='code-as-room')
    p.add_argument('--scene-usd',type=Path,help='Separate USD scene variant; code-as-room mode only')
    p.add_argument('--seconds',type=float,default=300.)
    p.add_argument('--output',type=Path)
    p.add_argument('--isaac-python',type=Path,default=Path(os.environ.get('ISAACSIM_PYTHON','D:/isaacsim/python.bat')))
    p.add_argument('--pixi',type=Path,help='Pixi executable; defaults to PIXI_EXE, PATH, or local research/runtime/pixi/pixi.exe')
    p.add_argument('--dynamic-test',action='store_true')
    p.add_argument('--range-noise',type=float,default=0.)
    p.add_argument('--domain-id',type=int,default=None)
    p.add_argument('--controller',choices=['rpp','dwb'],default='dwb')
    a=p.parse_args()
    if a.scene_usd and (a.scene!='code-as-room' or not a.scene_usd.is_file()):
        p.error('--scene-usd must be an existing file in code-as-room mode')
    out=(a.output or MODULES/'wheeled_navigation/artifacts'/f'autonomy_{a.stage}_{datetime.now():%Y%m%d_%H%M%S}').resolve()
    if (out/'status.json').exists():p.error('Choose a new output directory; existing runs are preserved')
    out.mkdir(parents=True,exist_ok=True)
    domain=a.domain_id if a.domain_id is not None else 5+secrets.randbelow(80)
    (out/'run_config.json').write_text(json.dumps({'ros_domain_id':domain,'arguments':vars(a)},default=str,indent=2))
    simulator=MODULES/'wheeled_sim_bridge/scripts/run_online_exploration.py'
    ros=None;logs=[]
    try:
        if a.stage=='ros2':
            pixi=a.pixi or Path(os.environ.get('PIXI_EXE') or shutil.which('pixi') or WORKSPACE/'research/runtime/pixi/pixi.exe')
            manifest=WORKSPACE/'research/ros2_runtime/pixi.toml'
            if not pixi.is_file():raise FileNotFoundError('Install the project ROS runtime first; see ONLINE_EXPLORATION.zh.md')
            roslog=(out/'ros_console.log').open('w',encoding='utf-8');logs.append(roslog)
            ros_env_vars=os.environ.copy();ros_env_vars['WHEELED_NAV_CONTROLLER']=a.controller
            ros=subprocess.Popen([str(pixi),'run','--manifest-path',str(manifest),'python',
                str(MODULES/'wheeled_navigation/scripts/ros_frontier_explorer.py'),
                '--launch-stack','--output',str(out/'ros'),'--limit',str(max(30.,a.seconds-60.)),
                '--wall-limit',str(max(900.,a.seconds*4)), '--sim-status-file',str(out/'status.json'),
                '--domain-id',str(domain)],
                cwd=WORKSPACE,stdout=roslog,stderr=subprocess.STDOUT,env=ros_env_vars)
        simlog=(out/'isaac.log').open('w',encoding='utf-8');logs.append(simlog)
        command=[str(a.isaac_python),str(simulator),'--mode',a.stage,'--scene',a.scene,
                 '--seconds',str(a.seconds),'--output',str(out)]
        if a.scene_usd:command+=['--scene-usd',str(a.scene_usd.resolve())]
        if a.dynamic_test:command.append('--dynamic-test')
        command+=['--range-noise',str(a.range_noise),'--domain-id',str(domain)]
        print('Output:',out,flush=True)
        print('Starting Isaac with live online decisions; no route file is accepted.',flush=True)
        # Isaac uses Python 3.10; RoboStack uses Python 3.12. Never let the
        # launcher's activated ROS/Conda Python paths leak into the simulator.
        sim_env=os.environ.copy()
        for key in ('PYTHONPATH','PYTHONHOME','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH',
                    'COLCON_PREFIX_PATH','CONDA_PREFIX','CONDA_DEFAULT_ENV','ROS_DISTRO','ROS_VERSION'):
            sim_env.pop(key,None)
        ros_env=str(WORKSPACE/'research/ros2_runtime/.pixi').lower().replace('\\','/')
        sim_env['PATH']=os.pathsep.join(q for q in sim_env.get('PATH','').split(os.pathsep)
                                      if ros_env not in q.lower().replace('\\','/'))
        sim=subprocess.Popen(command,cwd=WORKSPACE,stdout=simlog,stderr=subprocess.STDOUT,env=sim_env)
        while sim.poll() is None:
            time.sleep(5.)
            progress=out/'progress.json'
            if progress.is_file():
                try:
                    q=json.loads(progress.read_text());print(q['seconds'],q['phase'],q['truth'],flush=True)
                except (ValueError,KeyError):pass
            if ros and ros.poll() is not None and not (out/'ros/result.json').exists():
                raise RuntimeError('ROS exited before producing results; inspect ros_console.log')
        if not (out/'status.json').is_file():raise RuntimeError('Isaac exited without a result')
        result=json.loads((out/'status.json').read_text());print(json.dumps(result,indent=2),flush=True)
        if result.get('status')=='failed':raise RuntimeError(result['error'])
        if ros:
            try:ros.wait(timeout=8.)
            except subprocess.TimeoutExpired:pass
    finally:
        if 'sim' in locals() and sim.poll() is None:
            subprocess.run(['taskkill','/PID',str(sim.pid),'/T','/F'],capture_output=True)
        if ros and ros.poll() is None:
            subprocess.run(['taskkill','/PID',str(ros.pid),'/T','/F'],capture_output=True)
        for handle in logs:handle.close()

if __name__=='__main__':main()
