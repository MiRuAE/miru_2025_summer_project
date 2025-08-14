import rclpy, math, csv, time, numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

def quat_from_yaw(y):
    return Quaternion(x=0.0, y=0.0, z=math.sin(y/2.0), w=math.cos(y/2.0))

def arclen(xy):
    ds = np.linalg.norm(np.diff(xy, axis=0, append=xy[:1]), axis=1)
    s = np.cumsum(ds); s[-1] = s[-2] + ds[-1]
    s = np.insert(s, 0, 0.0)[:-1]  # start at 0
    return s, ds

def moving_average(x, k):
    # circular moving average
    n = len(x)
    pad = k//2
    xp = np.concatenate([x[-pad:], x, x[:pad]])
    kernel = np.ones(k)/k
    xs = np.convolve(xp, kernel, mode='valid')[:n]
    return xs

def curvature(xy):
    # discrete curvature using three-point formula on a closed loop
    p_prev = np.roll(xy, 1, axis=0)
    p_next = np.roll(xy, -1, axis=0)
    v1 = xy - p_prev
    v2 = p_next - xy
    a1 = np.arctan2(v1[:,1], v1[:,0])
    a2 = np.arctan2(v2[:,1], v2[:,0])
    dtheta = (a2 - a1 + np.pi) % (2*np.pi) - np.pi
    s = np.linalg.norm(v1, axis=1)
    s[s<1e-3] = 1e-3
    k = dtheta / s
    return k

class PathFollowPP(Node):
    def __init__(self):
        super().__init__('path_follow_pp')
        self._last_warn = 0.0
        # params
        self.declare_parameter('csv_path', 'raceline_raw.csv')
        self.declare_parameter('frame_map', 'map')
        self.declare_parameter('frame_base', 'ego_racecar/base_link')
        self.declare_parameter('path_topic', '/raceline')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('wheelbase', 0.33)     # F1TENTH 대략
        self.declare_parameter('lookahead', 1.0)      # 고정 Ld (m)
        self.declare_parameter('publish_path', True)
        self.declare_parameter('v_max', 3.0)
        self.declare_parameter('v_min', 0.5)
        self.declare_parameter('a_lat_max', 3.0)      # 횡가속 한계 (m/s^2)
        self.declare_parameter('smooth_window', 9)    # 홀수 권장

        self.frame_map  = self.get_parameter('frame_map').get_parameter_value().string_value
        self.frame_base = self.get_parameter('frame_base').get_parameter_value().string_value
        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.drive_topic= self.get_parameter('drive_topic').get_parameter_value().string_value
        
        self.L   = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.Ld  = self.get_parameter('lookahead').get_parameter_value().double_value
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self.v_min = self.get_parameter('v_min').get_parameter_value().double_value
        self.a_lat_max = self.get_parameter('a_lat_max').get_parameter_value().double_value
        self.pub_path_flag = self.get_parameter('publish_path').get_parameter_value().bool_value
        self.window = int(self.get_parameter('smooth_window').get_parameter_value().double_value) | 1

        self.tf_buf = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_ls  = TransformListener(self.tf_buf, self)
        self.pub_path = self.create_publisher(Path, self.path_topic, 1)
        self.pub_cmd  = self.create_publisher(AckermannDriveStamped, self.drive_topic, 1)

        # load path
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        pts = self.load_csv(csv_path)
        if pts.shape[0] < 5:
            raise RuntimeError("Not enough points in path CSV")

        # smooth & resample
        xy = pts[:,:2]
        # 스무딩
        xs = moving_average(xy[:,0], self.window)
        ys = moving_average(xy[:,1], self.window)
        xy = np.stack([xs, ys], axis=1)

        # 일정 간격 리샘플
        s, ds = arclen(xy)
        total = s[-1] + np.linalg.norm(xy[0]-xy[-1])
        step = 0.2  # 20cm로 리샘플
        s_new = np.arange(0.0, total, step)
        # 보간 (루프 보정)
        xy_loop = np.vstack([xy, xy[0]])
        s_loop, _ = arclen(xy_loop[:-1])
        x_new = np.interp(s_new, s_loop, xy_loop[:-1,0])
        y_new = np.interp(s_new, s_loop, xy_loop[:-1,1])
        self.path_xy = np.stack([x_new, y_new], axis=1)
        self.kappa = curvature(self.path_xy)
        self.N = self.path_xy.shape[0]

        # build Path msg
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_map
        for i in range(self.N):
            ps = PoseStamped()
            ps.header.frame_id = self.frame_map
            ps.pose.position.x = self.path_xy[i,0]
            ps.pose.position.y = self.path_xy[i,1]
            # yaw from forward difference
            j = (i+1) % self.N
            yaw = math.atan2(self.path_xy[j,1]-self.path_xy[i,1],
                             self.path_xy[j,0]-self.path_xy[i,0])
            ps.pose.orientation = quat_from_yaw(yaw)
            self.path_msg.poses.append(ps)

        if self.pub_path_flag:
            self.pub_path.publish(self.path_msg)
        self.timer = self.create_timer(0.02, self.control_step)  # 50 Hz

        self.get_logger().info(f"[PP] Loaded path: {self.N} pts, Ld={self.Ld} m, v_max={self.v_max} m/s")



    def load_csv(self, path):
        xs, ys = [], []
        with open(path, 'r') as f:
            rdr = csv.DictReader(f)
            for row in rdr:
                xs.append(float(row['x'])); ys.append(float(row['y']))
        arr = np.stack([np.array(xs), np.array(ys)], axis=1)
        return arr

    def warn_rl(self, msg, period=2.0):
        now = time.time()
        if now - self._last_warn >= period:
            self.get_logger().warn(msg)
            self._last_warn = now
        
    def control_step(self):
        # 현재 base_link pose in map
        try:
            tf = self.tf_buf.lookup_transform(self.frame_map, self.frame_base, rclpy.time.Time())
        except Exception as e:
            alt = 'base_link' if self.frame_base != 'base_link' else 'ego_racecar/base_link'
            try:
                tf = self.tf_buf.lookup_transform(self.frame_map, alt, rclpy.time.Time())
            # 성공하면 이 프레임으로 계속 쓰도록 업데이트(한 번만)
                self.warn_rl(f"TF frame '{self.frame_base}' 없어서 '{alt}'로 대체합니다.")
                self.frame_base = alt
            except Exception as e2:
            # 3) 여전히 실패 → 주기 제한 경고 후 리턴
                self.warn_rl(f"TF {self.frame_map}->{self.frame_base} 준비 안 됨: {e2}")
                return

        bx = tf.transform.translation.x
        by = tf.transform.translation.y
        # yaw from quat
        q = tf.transform.rotation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

        # 최근점 찾기
        d = np.hypot(self.path_xy[:,0]-bx, self.path_xy[:,1]-by)
        i_near = int(np.argmin(d))

        # Ld 떨어진 목표점 찾기(전방 진행)
        Ld = self.Ld
        # arc 누적
        idx = i_near
        acc = 0.0
        while acc < Ld:
            j = (idx + 1) % self.N
            acc += math.hypot(self.path_xy[j,0]-self.path_xy[idx,0],
                              self.path_xy[j,1]-self.path_xy[idx,1])
            idx = j
        target = self.path_xy[idx]

        # 타겟까지의 로컬 좌표
        dx = target[0] - bx
        dy = target[1] - by
        # 차량 좌표계로 회전
        tx =  math.cos(-yaw)*dx - math.sin(-yaw)*dy
        ty =  math.sin(-yaw)*dx + math.cos(-yaw)*dy

        # Pure Pursuit: kappa = 2*ty / Ld^2, steer = atan(kappa * L)
        if Ld < 1e-3: return
        kappa_pp = 2.0*ty/(Ld*Ld)
        steer = math.atan(kappa_pp * self.L)

        # 속도: 곡률 제한 + 상한/하한
        k_here = abs(self.kappa[i_near])
        v_lat = math.sqrt(max(self.a_lat_max,1e-3) / (k_here + 1e-6))
        v_cmd = max(self.v_min, min(self.v_max, v_lat))

        msg = AckermannDriveStamped()
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(v_cmd)
        self.pub_cmd.publish(msg)

def main():
    rclpy.init()
    node = PathFollowPP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

