"""Optional live publisher using Isaac Sim's bundled ROS 2 Humble runtime.

The external SLAM mode publishes no map->odom TF or /map, so SLAM Toolbox
owns those. No ROS dependency is imported by the native demo unless requested.
"""
import math


class Ros2SensorBridge:
    def __init__(self, external_slam=False, base_height=.45, lidar_mount=(0,0,.92), max_range=20.):
        import rclpy
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry, OccupancyGrid, Path
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("wheeled_isaac_sensor_bridge")
        self.external_slam = external_slam
        self.max_range = float(max_range)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.scan = self.node.create_publisher(LaserScan, "/scan", 10)
        self.odom = self.node.create_publisher(Odometry, "/odom", 10)
        self.clock = self.node.create_publisher(Clock, "/clock", 10)
        self.tf = self.node.create_publisher(TFMessage, "/tf", 10)
        self.tf_static = self.node.create_publisher(TFMessage, "/tf_static", latched)
        self.map = self.node.create_publisher(OccupancyGrid, "/map", latched) if not external_slam else None
        self.path = self.node.create_publisher(Path, "/native_demo/plan", latched)
        self.last = None
        self.received = {"scan": 0, "odom": 0, "map": 0}
        self.scan_check = self.node.create_subscription(LaserScan, "/scan", lambda _: self._received("scan"), 10)
        self.odom_check = self.node.create_subscription(Odometry, "/odom", lambda _: self._received("odom"), 10)
        self.map_check = self.node.create_subscription(OccupancyGrid, "/map", lambda _: self._received("map"), latched)
        self.last_command = (0.0, 0.0)
        self.command_received_time = None
        from geometry_msgs.msg import Twist
        self.command_sub = self.node.create_subscription(Twist, "/cmd_vel", self._command_callback, 10)
        static = TFMessage()
        static.transforms = [self.transform("base_footprint", "base_link", 0, 0, base_height, 0, 0),
                             self.transform("base_link", "lidar_link", lidar_mount[0],lidar_mount[1],
                                            lidar_mount[2]-base_height,0,0)]
        self.tf_static.publish(static)
        self.static_message = static
        self.static_last_sent = None

    @staticmethod
    def stamp(seconds):
        from builtin_interfaces.msg import Time
        sec = int(seconds)
        return Time(sec=sec, nanosec=int((seconds-sec)*1e9))

    def transform(self, parent, child, x, y, z, yaw, seconds):
        from geometry_msgs.msg import TransformStamped
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.header.stamp = self.stamp(seconds)
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = float(x), float(y), float(z)
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
        return t

    def publish(self, seconds, pose, angles, ranges):
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        import numpy as np
        self.clock.publish(Clock(clock=self.stamp(seconds)))
        # Republish the same fixed mount for late-joining nodes across Isaac's
        # bundled FastDDS and the independent Windows ROS runtime.
        if self.static_last_sent is None or seconds-self.static_last_sent>=1.:
            self.tf_static.publish(self.static_message)
            self.static_last_sent=seconds
        order = np.argsort(angles)
        ordered = np.asarray(angles)[order]
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = self.stamp(seconds), "lidar_link"
        scan.angle_min, scan.angle_max = float(ordered[0]), float(ordered[-1])
        scan.angle_increment = float((ordered[-1]-ordered[0])/(len(ordered)-1))
        scan.range_min, scan.range_max = 0.15, self.max_range
        scan.scan_time = float(seconds-self.last[0]) if self.last else 0.05
        # PhysX rotation_rate=0 gives one instantaneous full scan; no per-beam skew.
        scan.time_increment = 0.0
        scan.ranges = [float(r) if r < self.max_range-.01 else float("inf") for r in np.asarray(ranges)[order]]
        self.scan.publish(scan)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = self.stamp(seconds), "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(pose[0]), float(pose[1])
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
        for i,value in [(0,.0025),(7,.0025),(14,.01),(21,.01),(28,.01),(35,.001)]:
            odom.pose.covariance[i] = value
            odom.twist.covariance[i] = value
        if self.last and seconds > self.last[0]:
            t, old = self.last
            dt = seconds-t
            dx,dy = pose[0]-old[0],pose[1]-old[1]
            odom.twist.twist.linear.x = (math.cos(pose[2])*dx+math.sin(pose[2])*dy)/dt
            odom.twist.twist.linear.y = (-math.sin(pose[2])*dx+math.cos(pose[2])*dy)/dt
            odom.twist.twist.angular.z = math.atan2(math.sin(pose[2]-old[2]),math.cos(pose[2]-old[2]))/dt
        self.odom.publish(odom)
        transforms = [self.transform("odom", "base_footprint", *pose[:2], 0, pose[2], seconds)]
        if not self.external_slam:
            transforms.append(self.transform("map", "odom", 0,0,0,0,seconds))
        self.tf.publish(TFMessage(transforms=transforms))
        self.last = seconds, tuple(pose)
        self.spin_once()

    def publish_map(self, mapper, seconds):
        if self.map is None:
            return
        import numpy as np
        from nav_msgs.msg import OccupancyGrid
        s = mapper.spec
        msg = OccupancyGrid()
        msg.header.stamp, msg.header.frame_id = self.stamp(seconds), "map"
        msg.info.resolution, msg.info.width, msg.info.height = s.resolution, s.width, s.height
        msg.info.origin.position.x, msg.info.origin.position.y = s.xmin, s.ymin
        msg.info.origin.orientation.w = 1.0
        data = np.full(mapper.observed.shape, -1, dtype=np.int8)
        data[mapper.free()] = 0
        data[mapper.occupied()] = 100
        msg.data = data.ravel().tolist()
        self.map.publish(msg)

    def publish_path(self, path, seconds):
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        msg = Path()
        msg.header.stamp, msg.header.frame_id = self.stamp(seconds), "map"
        for x,y in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path.publish(msg)

    def _command_callback(self, msg):
        import time
        self.last_command = float(msg.linear.x), float(msg.angular.z)
        self.command_received_time = time.monotonic()

    def _received(self, kind):
        self.received[kind] += 1

    def command(self):
        """Use wall time for stale commands even when simulation is paused."""
        import time
        if self.command_received_time is None or time.monotonic()-self.command_received_time > 0.5:
            return 0.0, 0.0
        vx,wz = self.last_command
        return max(-0.3,min(.6,vx)), max(-.7,min(.7,wz))

    def spin_once(self):
        for _ in range(8):
            self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()
