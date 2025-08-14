import rclpy, math, csv, os
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from rclpy.duration import Duration

def yaw_from_quat(q):
    # q: geometry_msgs/Quaternion
    # yaw-only
    siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class LapRecorder(Node):
    def __init__(self):
        super().__init__('lap_recorder')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('csv_path', 'raceline_raw.csv')
        self.declare_parameter('frame_map', 'map')
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('sample_dist', 0.20)  # 20cm 간격

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.csv_path   = self.get_parameter('csv_path').get_parameter_value().string_value
        self.frame_map  = self.get_parameter('frame_map').get_parameter_value().string_value
        self.frame_odom = self.get_parameter('frame_odom').get_parameter_value().string_value
        self.sample_dist = self.get_parameter('sample_dist').get_parameter_value().double_value

        self.tf_buf = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_ls  = TransformListener(self.tf_buf, self)

        self.sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)

        self.points = []  # (x, y, yaw)
        self.last_xy = None
        self.get_logger().info(f"[LapRecorder] Recording {self.odom_topic} → {self.csv_path} (frame={self.frame_map})")

    def odom_cb(self, msg: Odometry):
        # odom pose → map pose 변환
        try:
            tf = self.tf_buf.lookup_transform(self.frame_map, msg.header.frame_id, rclpy.time.Time())
        except Exception as e:
            self.get_logger().throttle(self.get_clock(), 2000, f"TF {self.frame_map}<-{msg.header.frame_id} not ready: {e}")
            return

        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        pm = do_transform_pose(ps.pose, tf)

        x = pm.position.x
        y = pm.position.y
        yaw = yaw_from_quat(pm.orientation)

        if self.last_xy is None or math.hypot(x - self.last_xy[0], y - self.last_xy[1]) >= self.sample_dist:
            self.points.append((x, y, yaw))
            self.last_xy = (x, y)

    def destroy_node(self):
        # 저장
        if self.points:
            path = os.path.expanduser(self.csv_path)
            os.makedirs(os.path.dirname(path), exist_ok=True) if os.path.dirname(path) else None
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['x','y','yaw'])
                for p in self.points:
                    w.writerow([f"{p[0]:.6f}", f"{p[1]:.6f}", f"{p[2]:.6f}"])
            self.get_logger().info(f"[LapRecorder] Saved {len(self.points)} points → {path}")
        super().destroy_node()

def main():
    rclpy.init()
    node = LapRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

