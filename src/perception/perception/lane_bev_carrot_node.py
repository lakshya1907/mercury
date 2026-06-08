"""
lane_bev_carrot_node.py  v4
---------------------------
Safety: TWO independent obstacle checks per candidate:
  1. road_costmap  (/perception/road_costmap, map frame, 5Hz)
     → rejects candidates outside lane boundaries (cost >= safe_cost_max)
  2. LaserScan     (/scan, real-time)
     → rejects candidates within min_clear_m of any scan hit
     → no costmap lag, no unknown-cell loophole
"""

import math, json, os
import numpy as np
import cv2
import rclpy, rclpy.duration, rclpy.time, rclpy.qos
from rclpy.node import Node
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from ament_index_python.packages import get_package_share_directory
import tf_transformations

_R_OPT = np.array([[0,0,1],[-1,0,0],[0,-1,0]], dtype=np.float64)

def _qrot(q):
    qx,qy,qz,qw = q.x,q.y,q.z,q.w
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),  2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),  1-2*(qx*qx+qz*qz),  2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ], dtype=np.float64)


class LaneBevCarrotNode(Node):

    def __init__(self):
        super().__init__('lane_bev_carrot')

        self.declare_parameter('carrot_dist_m',          2.0)
        self.declare_parameter('goal_tolerance',          0.8)
        self.declare_parameter('publish_rate',            5.0)
        self.declare_parameter('camera_hfov',             1.047)
        self.declare_parameter('image_width',             640)
        self.declare_parameter('image_height',            480)
        self.declare_parameter('min_proj_m',              0.2)
        self.declare_parameter('max_proj_m',              6.0)
        self.declare_parameter('n_bev_samples',           50)
        self.declare_parameter('fit_cache_sec',           1.0)
        self.declare_parameter('no_carrot_stop_streak',   3)
        self.declare_parameter('safe_cost_max',           50)
        self.declare_parameter('min_clear_m',             0.6)
        self.declare_parameter('safety_radius', 0.30)
        self.declare_parameter('max_carrot_dist_m', 4.0)


        p = lambda n: self.get_parameter(n).value
        self._carrot_dist     = float(p('carrot_dist_m'))
        self._goal_tol        = float(p('goal_tolerance'))
        rate                  = float(p('publish_rate'))
        hfov                  = float(p('camera_hfov'))
        img_w                 = int(p('image_width'))
        img_h                 = int(p('image_height'))
        self._min_proj        = float(p('min_proj_m'))
        self._max_proj        = float(p('max_proj_m'))
        self._n_samples       = int(p('n_bev_samples'))
        self._fit_cache_sec   = float(p('fit_cache_sec'))
        self._stop_max        = int(p('no_carrot_stop_streak'))
        self._safe_cost_max   = int(p('safe_cost_max'))
        self._min_clear_m     = float(p('min_clear_m'))
        self._safety_r        = float(p('safety_radius'))   # ← add here
        self._max_carrot_dist = float(p('max_carrot_dist_m'))
        

        self._fx = (img_w/2.0)/math.tan(hfov/2.0)
        self._cx = img_w/2.0
        self._cy = img_h/2.0

        pkg = get_package_share_directory('perception')
        def _load(n): return json.load(open(os.path.join(pkg,'config',n)))
        bev  = _load('bev_config.json')
        road = _load('road_config.json')
        sw   = _load('sliding_window_config.json')

        

        src = np.float32(bev['src_points'])
        dst = np.float32(bev['dst_points'])
        self._M     = cv2.getPerspectiveTransform(src, dst)
        self._M_inv = cv2.getPerspectiveTransform(dst, src)
        self._bev_w = int(np.max(dst[:,0]))
        self._bev_h = int(np.max(dst[:,1]))
        self._road_v_min = int(road['v_min'])
        self._road_v_max = int(road['v_max'])
        self._road_s_max = int(road.get('s_max',255))
        self._win_h = max(1, int(sw['window_height']))

        self._last_fit_robot = (0.0, 0.0)

        self._carrot_locked  = False
        self._locked_carrot  = None  # (wx, wy)

        # state
        self._final_goal      = None
        self._robot_x = self._robot_y = self._robot_yaw = 0.0
        self._last_img        = None
        self._last_fit        = None
        self._last_fit_stamp  = None
        self._streak          = 0

        # road costmap (map frame, lane boundaries)
        self._road_grid = None
        self._road_info = None

        # laser scan points in map frame (Nx2 numpy array or None)
        self._scan_pts_map: np.ndarray | None = None

        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)
        self._bridge = CvBridge()

        sq = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        lq = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(PoseStamped,  '/final_goal',                   self._goal_cb,    10)
        self.create_subscription(Odometry,     '/diff_drive_controller/odom',   self._odom_cb,    10)
        self.create_subscription(Image,        '/camera/image_raw',             self._img_cb,     sq)
        self.create_subscription(OccupancyGrid,'/perception/road_costmap',      self._road_cb,    lq)
        self.create_subscription(LaserScan,    '/scan',                         self._scan_cb,    sq)

        self._pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_timer(1.0/rate, self._tick)
        self.get_logger().info(
            f'LaneBevCarrotNode v4 | safe_cost={self._safe_cost_max} '
            f'min_clear={self._min_clear_m}m')

    # ── callbacks ──────────────────────────────────────────────────────

    def _goal_cb(self, msg):
        self._final_goal = msg; 
        self._streak = 0
        self._carrot_locked = False
        self._locked_carrot = None

    def _odom_cb(self, msg):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _,_,self._robot_yaw = tf_transformations.euler_from_quaternion(
            [q.x,q.y,q.z,q.w])

    def _img_cb(self, msg):
        try: self._last_img = self._bridge.imgmsg_to_cv2(msg,'bgr8')
        except: pass

    def _road_cb(self, msg: OccupancyGrid):
        self._road_info = msg.info
        self._road_grid = msg.data

    def _scan_cb(self, msg: LaserScan):
        """Convert scan to map-frame point cloud and cache it."""
        try:
            tf = self._tf_buf.lookup_transform(
                'map', msg.header.frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException:
            return

        R = _qrot(tf.transform.rotation)
        t = tf.transform.translation

        pts = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                p = R @ np.array([x, y, 0.0])
                pts.append((p[0]+t.x, p[1]+t.y))
            angle += msg.angle_increment

        self._scan_pts_map = np.array(pts, dtype=np.float64) if pts else None

    # ── safety checks ──────────────────────────────────────────────────

    def _road_cost(self, wx, wy) -> int:
        if self._road_grid is None: return -1
        info = self._road_info
        col = int((wx - info.origin.position.x) / info.resolution)
        row = int((wy - info.origin.position.y) / info.resolution)
        if not (0 <= col < info.width and 0 <= row < info.height): return -1
        return int(self._road_grid[row * info.width + col])

    def _is_safe(self, wx, wy) -> bool:
        check_pts = [(wx, wy)]
        for deg in (0, 90, 180, 270, 45, 135, 225, 315):
            a = math.radians(deg)
            check_pts.append((wx + self._safety_r * math.cos(a),
                            wy + self._safety_r * math.sin(a)))
        for px, py in check_pts:
            c = self._road_cost(px, py)
            if c != -1 and c >= self._safe_cost_max:
                return False
        if self._scan_pts_map is not None and len(self._scan_pts_map) > 0:
            dists = np.hypot(self._scan_pts_map[:, 0] - wx,
                            self._scan_pts_map[:, 1] - wy)
            if np.any(dists < self._min_clear_m):
                return False
        return True

    # ── BEV road fit ───────────────────────────────────────────────────

    def _road_fit(self, bev):
        hsv  = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
            np.array([0,0,self._road_v_min]),
            np.array([180,self._road_s_max,self._road_v_max]))
        k = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        h = mask.shape[0]
        xs,ys = [],[]
        y = h
        while y > 0:
            y0 = max(0, y-self._win_h)
            cols = np.where(np.any(mask[y0:y,:]>0, axis=0))[0]
            if len(cols) >= 2:
                xs.append(int((int(cols[0])+int(cols[-1]))/2))
                ys.append((y0+y)//2)
            y -= self._win_h
        return np.polyfit(ys,xs,2) if len(xs)>=3 else None

    # ── projection ─────────────────────────────────────────────────────

    def _bev_to_ground(self, u_bev, v_bev, cam_pos, R_cam):
        pt = cv2.perspectiveTransform(
            np.array([[[u_bev,v_bev]]],dtype=np.float32), self._M_inv)[0,0]
        ray = R_cam @ (_R_OPT @ np.array(
            [(pt[0]-self._cx)/self._fx,
             (pt[1]-self._cy)/self._fx, 1.0]))
        if ray[2] >= -1e-4: return None
        lam = -cam_pos[2]/ray[2]
        if lam <= 0: return None
        wx = cam_pos[0]+lam*ray[0]; wy = cam_pos[1]+lam*ray[1]
        if not (self._min_proj <= math.hypot(wx-cam_pos[0],wy-cam_pos[1]) <= self._max_proj):
            return None
        return wx, wy
    
    def _lateral_clearance(self, wx, wy) -> float:
        """Returns min distance (in metres) to nearest lethal cell, capped at 2.0m."""
        best = 2.0
        step = 0.1
        for r in np.arange(step, 2.0, step):
            for deg in (0, 45, 90, 135, 180, 225, 270, 315):
                a = math.radians(deg)
                c = self._road_cost(wx + r*math.cos(a), wy + r*math.sin(a))
                if c != -1 and c >= self._safe_cost_max:
                    best = min(best, r)
                    break
        return best

    def _publish_stop(self):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self._robot_x
        msg.pose.position.y = self._robot_y
        msg.pose.orientation.z = math.sin(self._robot_yaw/2)
        msg.pose.orientation.w = math.cos(self._robot_yaw/2)
        self._pub.publish(msg)
        self.get_logger().warn(f'STOP — no safe carrot for {self._streak} ticks')

    # ── main tick ──────────────────────────────────────────────────────

    def _tick(self):
        if self._final_goal is None or self._last_img is None:
            return
        gx = self._final_goal.pose.position.x
        gy = self._final_goal.pose.position.y
        if math.hypot(gx-self._robot_x, gy-self._robot_y) < self._goal_tol:
            self.get_logger().info('Goal reached!')
            self._final_goal = None; self._streak = 0; return

        try:
            cam_tf = self._tf_buf.lookup_transform(
                'map', 'camera_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException:
            return

        t       = cam_tf.transform.translation
        cam_pos = np.array([t.x, t.y, t.z])
        R_cam   = _qrot(cam_tf.transform.rotation)
        fwd     = np.array([math.cos(self._robot_yaw), math.sin(self._robot_yaw)])

        if self._carrot_locked and self._locked_carrot is not None:
            cx, cy = self._locked_carrot
            dp_from_cam = np.array([cx - cam_pos[0], cy - cam_pos[1]])
            if np.dot(fwd, dp_from_cam) <= 0 or not self._is_safe(cx, cy):
                self.get_logger().info('Locked carrot invalidated — recomputing')
                self._carrot_locked = False
                self._locked_carrot = None
            else:
                yaw = math.atan2(cy - self._robot_y, cx - self._robot_x)
                msg = PoseStamped()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                msg.pose.position.x = cx
                msg.pose.position.y = cy
                msg.pose.orientation.z = math.sin(yaw / 2)
                msg.pose.orientation.w = math.cos(yaw / 2)
                self._pub.publish(msg)
                return

        bev   = cv2.warpPerspective(self._last_img, self._M, (self._bev_w, self._bev_h))
        fresh = self._road_fit(bev)
        if fresh is not None:
            self._last_fit       = fresh
            self._last_fit_stamp = self.get_clock().now()
            self._last_fit_robot = (self._robot_x, self._robot_y)

        fit = self._last_fit
        if fit is not None and self._last_fit_stamp is not None:
            age   = (self.get_clock().now() - self._last_fit_stamp).nanoseconds / 1e9
            drift = math.hypot(self._robot_x - self._last_fit_robot[0],
                               self._robot_y - self._last_fit_robot[1])
            if age > self._fit_cache_sec or drift > 0.3:
                fit = None

        carrot   = None
        best_score = float('inf')

        if fit is not None:
            for v in np.linspace(self._bev_h-1, 0, self._n_samples):
                u  = float(np.clip(fit[0]*v**2 + fit[1]*v + fit[2], 0, self._bev_w-1))
                pt = self._bev_to_ground(u, v, cam_pos, R_cam)
                if pt is None: continue
                dp          = np.array([pt[0]-self._robot_x, pt[1]-self._robot_y])
                dist_to_pt  = math.hypot(*dp)
                dp_from_cam = np.array([pt[0]-cam_pos[0], pt[1]-cam_pos[1]])
                if np.dot(fwd, dp_from_cam) <= 0:     continue
                if dist_to_pt < self._min_proj:        continue
                if dist_to_pt > self._max_carrot_dist: continue
                if not self._is_safe(pt[0], pt[1]):    continue
                clearance = self._lateral_clearance(pt[0], pt[1])
                score = abs(dist_to_pt - self._carrot_dist) - 0.5 * clearance
                if score < best_score:
                    best_score = score; carrot = pt

        if carrot is None:
            self._streak += 1
            self.get_logger().warn(f'No safe carrot (streak={self._streak})',
                                   throttle_duration_sec=1.0)
            if self._streak >= self._stop_max:
                self._publish_stop()
            return

        if math.hypot(carrot[0]-gx, carrot[1]-gy) <= self._goal_tol:
            self._carrot_locked = True
            self._locked_carrot = carrot
            self.get_logger().info(f'Carrot locked at ({carrot[0]:.2f}, {carrot[1]:.2f})')

        self._streak = 0
        dx  = carrot[0] - self._robot_x
        dy  = carrot[1] - self._robot_y
        yaw = math.atan2(dy, dx)
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = carrot[0]
        msg.pose.position.y = carrot[1]
        msg.pose.orientation.z = math.sin(yaw/2)
        msg.pose.orientation.w = math.cos(yaw/2)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneBevCarrotNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()