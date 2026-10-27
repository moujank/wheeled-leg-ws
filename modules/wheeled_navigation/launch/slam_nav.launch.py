"""Native ROS 2 Humble SLAM Toolbox + Nav2, independent of Isaac Python."""
from pathlib import Path
import os
import tempfile
import yaml
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0,str(root))
    from scan_nav.footprint import ROBOT_POLYGON,PADDING
    nav_share = Path(get_package_share_directory("nav2_bringup"))
    slam_share = Path(get_package_share_directory("slam_toolbox"))
    params = yaml.safe_load((nav_share/"params/nav2_params.yaml").read_text())
    for entry in params.values():
        if isinstance(entry,dict) and "ros__parameters" in entry:
            entry["ros__parameters"]["use_sim_time"] = True
    for name in ("local_costmap","global_costmap"):
        cfg=params[name][name]["ros__parameters"]
        cfg.update(robot_base_frame="base_footprint",robot_radius=.38,
                   footprint_padding=PADDING,resolution=.025,use_sim_time=True,
                   update_frequency=5.,publish_frequency=2.,always_send_full_costmap=True)
        cfg["plugins"] = (["static_layer"] if name=="global_costmap" else []) + ["obstacle_layer","inflation_layer"]
        cfg['footprint']=str(ROBOT_POLYGON)
        cfg["obstacle_layer"]={"plugin":"nav2_costmap_2d::ObstacleLayer",
            "enabled":True,"observation_sources":"scan",
            # Fresh ray-traced observations can correct centimetre-scale static
            # SLAM edge errors. Only measured cells overwrite the prior; do not
            # clear the robot footprint merely because the robot is there.
            "combination_method":0 if name=='global_costmap' else 1,
            "footprint_clearing_enabled":False,
            "scan":{"topic":"/scan","data_type":"LaserScan","marking":True,"clearing":True,
                    "max_obstacle_height":1.5,"raytrace_max_range":7.9,"obstacle_max_range":7.8,
                    "inf_is_valid":True}}
        cfg["inflation_layer"].update(inflation_radius=.45,cost_scaling_factor=8.)
        cfg["transform_tolerance"] = .5
        if name=="local_costmap":
            cfg.update(width=4,height=4)
    ctrl=params["controller_server"]["ros__parameters"]
    ctrl.update(odom_topic="/odom",controller_frequency=10.,failure_tolerance=2.)
    ctrl["progress_checker"].update(required_movement_radius=.15,movement_time_allowance=25.)
    ctrl["general_goal_checker"].update(xy_goal_tolerance=.12,yaw_goal_tolerance=.5)
    ctrl["FollowPath"]={"plugin":"nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        "desired_linear_vel":.18,"lookahead_dist":.10,"min_lookahead_dist":.06,
        "max_lookahead_dist":.3,"lookahead_time":1.2,"use_velocity_scaled_lookahead_dist":False,
        "rotate_to_heading_angular_vel":.5,"transform_tolerance":.5,
        "min_approach_linear_velocity":.03,"approach_velocity_scaling_dist":.35,
        "max_allowed_time_to_collision_up_to_carrot":1.,"use_collision_detection":True,
        "use_regulated_linear_velocity_scaling":True,"use_cost_regulated_linear_velocity_scaling":True,
        "regulated_linear_scaling_min_radius":.4,"regulated_linear_scaling_min_speed":.03,
        "use_rotate_to_heading":False,"allow_reversing":True,"rotate_to_heading_min_angle":.5,"max_angular_accel":.8,
        "cost_scaling_dist":.3,"cost_scaling_gain":1.,"inflation_cost_scaling_factor":8.}
    if os.environ.get('WHEELED_NAV_CONTROLLER','dwb')=='dwb':
        ctrl['FollowPath']={
            'plugin':'dwb_core::DWBLocalPlanner','debug_trajectory_details':False,
            'min_vel_x':-.18,'min_vel_y':0.,'max_vel_x':.18,'max_vel_y':0.,
            'max_vel_theta':.7,'min_speed_xy':0.,'max_speed_xy':.18,'min_speed_theta':0.,
            'acc_lim_x':.5,'acc_lim_y':0.,'acc_lim_theta':1.8,
            'decel_lim_x':-.7,'decel_lim_y':0.,'decel_lim_theta':-1.8,
            'vx_samples':15,'vy_samples':1,'vtheta_samples':25,'sim_time':1.,
            'linear_granularity':.025,'angular_granularity':.04,
            'transform_tolerance':.5,'xy_goal_tolerance':.12,'trans_stopped_velocity':.02,
            'short_circuit_trajectory_evaluation':True,'stateful':True,
            'critics':['ObstacleFootprint','RotateToGoal','Oscillation','PathDist','GoalDist','PathAlign'],
            'ObstacleFootprint.scale':.04,'PathDist.scale':32.,'GoalDist.scale':24.,
            'PathAlign.scale':8.,'PathAlign.forward_point_distance':.1,
            'RotateToGoal.scale':32.,'RotateToGoal.slowing_factor':5.,
            'RotateToGoal.lookahead_time':-1.}
    params["planner_server"]["ros__parameters"]["GridBased"]={
        'plugin':'nav2_smac_planner/SmacPlannerHybrid','tolerance':.08,'allow_unknown':False,
        'downsample_costmap':False,'angle_quantization_bins':72,'minimum_turning_radius':.20,
        'motion_model_for_search':'REEDS_SHEPP','max_iterations':500000,'max_on_approach_iterations':1000,
        'max_planning_time':3.,'lookup_table_size':3.,'cache_obstacle_heuristic':False,
        'reverse_penalty':1.5,'change_penalty':.1,'non_straight_penalty':1.1,
        'cost_penalty':3.,'analytic_expansion_max_length':1.,'smooth_path':False}
    bt=params["bt_navigator"]["ros__parameters"]
    bt.update(robot_base_frame="base_footprint",transform_tolerance=.5,bt_loop_duration=50,default_server_timeout=100,
              default_nav_to_pose_bt_xml=(root/"config/explore_navigate.xml").as_posix(),
              default_nav_through_poses_bt_xml=(root/"config/explore_through_poses.xml").as_posix())
    behavior=params["behavior_server"]["ros__parameters"]
    behavior.update(robot_base_frame="base_footprint",behavior_plugins=["wait"])
    smoother=params["velocity_smoother"]["ros__parameters"]
    smoother.update(max_velocity=[.22,0.,.7],min_velocity=[-.18,0.,-.7],
                    max_accel=[.5,0.,1.8],max_decel=[-.7,0.,-1.8],smoothing_frequency=10.)
    with tempfile.NamedTemporaryFile(mode="w",suffix="_wheeled_nav.yaml",delete=False) as f:
        yaml.safe_dump(params,f); nav_params=f.name
    nodes=[('nav2_controller','controller_server'),('nav2_planner','planner_server'),
           ('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator'),
           ('nav2_velocity_smoother','velocity_smoother')]
    navigation=[]
    for package,name in nodes:
        remaps=[('/tf','tf'),('/tf_static','tf_static')]
        if name=='controller_server':remaps += [('cmd_vel','cmd_vel_nav')]
        if name=='velocity_smoother':remaps += [('cmd_vel','cmd_vel_nav'),('cmd_vel_smoothed','cmd_vel')]
        navigation.append(Node(package=package,executable=name,name=name,output='screen',
            parameters=[nav_params],remappings=remaps,
            arguments=['--ros-args','--log-level',os.environ.get('WHEELED_NAV_LOG_LEVEL','info')]))
    navigation.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        name='lifecycle_manager_navigation',output='screen',
        parameters=[{'use_sim_time':True,'autostart':True,'node_names':[name for _,name in nodes]}]))
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM','0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam_share/"launch/online_async_launch.py")),
            launch_arguments={"use_sim_time":"true","slam_params_file":str(root/"config/slam_toolbox.yaml")}.items()),
        *navigation])
