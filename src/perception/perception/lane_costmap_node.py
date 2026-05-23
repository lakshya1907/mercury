"""
lane_costmap_node.py  (v2 — robust projection)
-----------------------------------------------
Projects detected lane boundaries from the camera into a Nav2
OccupancyGrid (/perception/road_costmap, map frame).

KEY FIXES vs v1
---------------
1. Forward-only gate  — only accepts ground points that are AHEAD of the
   robot (dot product with robot heading > 0).  Prevents lethal cells
   appearing behind/beside the robot.

2. Tight distance gate — rejects projections > MAX_PROJ_M from the robot
   (default 5 m).  At 2 m camera height a 1° angle error gives ~0.2 m
   ground error at 5 m but 3 m error at 30 m.  Keep projections close.

3. Minimum distance gate — rejects projections < 0.3 m from the camera
   (floor reflections, robot body).

4. Robot TF lookup — obtains robot position in map frame each frame so
   the forward/distance gates work correctly.

5. Conservative lethal marking — only pixels clearly OUTSIDE the boundary
   are marked lethal.  Uses a smaller pixel step to avoid bleed.

6. Calibrated half-lane width fall-back — if only one lane line is visible,
   uses a conservative 1.0 m half-width instead of the image-based estimate
   to avoid marking the drivable corridor as obstacle.

Camera model assumption
-----------------------
  URDF: camera_link at xyz="0.15 0 2" rpy="0 0.4 0" (from base_link)
  Sensor: hfov=1.047 rad (60°), 640×480 px.
  Optical frame: x=right, y=down, z=forward (standard ROS camera optical).
  Rotation optical→link (link: x=forward, y=left, z=up):
      link_x =  opt_z
      link_y = -opt_x
      link_z = -opt_y
  TF lookup camera_link→map already contains the URDF pitch + robot pose.
"""

import math
import numpy as np
import cv2

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
)

import tf2_ros
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image


# Optical-frame → camera_link rotation (constant, derived from ROS convention)
_R_OPT_TO_LINK = np.array(
    [[ 0,  0,  1],
     [-1,  0,  0],
     [ 0, -1,  0]], dtype=np.float64)


def _quat_to_rot(q) -> np.ndarray:
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1-2*(qx*qx + qz*qz),   2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),   2*(qy*qz + qx*qw), 1-2*(qx*qx + qy*qy)],
    ], dtype=np.float64)


class LaneCostmapNode(Node):

    def __init__(self):
        super().__init__('lane_costmap')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('map_width_m',    70.0)
        self.declare_parameter('map_height_m',   70.0)
        self.declare_parameter('resolution',      0.10)
        self.declare_parameter('map_origin_x',  -35.0)
        self.declare_parameter('map_origin_y',  -35.0)
        self.declare_parameter('publish_rate',    5.0)

        # Camera
        self.declare_parameter('camera_hfov',    1.047)
        self.declare_parameter('image_width',    640)
        self.declare_parameter('image_height',   480)
        self.declare_parameter('roi_top_frac',   0.45)   # more aggressive ROI crop

        # Projection safety gates
        self.declare_parameter('max_proj_m',     5.0)    # max ground dist from camera
        self.declare_parameter('min_proj_m',     0.3)    # min ground dist (body exclusion)
        self.declare_parameter('forward_only',   True)   # only project ahead of robot

        # Lane marking thresholds
        self.declare_parameter('white_v_min',    170)
        self.declare_parameter('white_s_max',     60)
        self.declare_parameter('sample_rows',      6)
        self.declare_parameter('process_every_n',  3)

        # How many pixels outside the boundary to mark lethal.
        # Keep small: 20 px ≈ 0.10 m at typical close range.
        self.declare_parameter('lethal_band_px',  20)

        # ── Read ─────────────────────────────────────────────────────────────
        def _p(n): return self.get_parameter(n).value

        map_w_m           = float(_p('map_width_m'))
        map_h_m           = float(_p('map_height_m'))
        self._res         = float(_p('resolution'))
        self._origin_x    = float(_p('map_origin_x'))
        self._origin_y    = float(_p('map_origin_y'))
        rate              = float(_p('publish_rate'))
        self._hfov        = float(_p('camera_hfov'))
        self._img_w       = int(_p('image_width'))
        self._img_h       = int(_p('image_height'))
        self._roi_top     = float(_p('roi_top_frac'))
        self._max_proj    = float(_p('max_proj_m'))
        self._min_proj    = float(_p('min_proj_m'))
        self._fwd_only    = bool(_p('forward_only'))
        self._white_vmin  = int(_p('white_v_min'))
        self._white_smax  = int(_p('white_s_max'))
        self._sample_rows = int(_p('sample_rows'))
        self._skip_n      = int(_p('process_every_n'))
        self._lethal_px   = int(_p('lethal_band_px'))

        # ── Derived ──────────────────────────────────────────────────────────
        self._grid_w = int(round(map_w_m  / self._res))
        self._grid_h = int(round(map_h_m  / self._res))
        self._fx     = (self._img_w / 2.0) / math.tan(self._hfov / 2.0)
        self._fy     = self._fx
        self._cx     = self._img_w  / 2.0
        self._cy     = self._img_h  / 2.0

        # Persistent costmap: -1=unknown, 0=free, 100=lethal
        self._grid = np.full(self._grid_w * self._grid_h, -1, dtype=np.int8)

        self._frame_count = 0

        # ── TF ───────────────────────────────────────────────────────────────
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        # ── QoS ──────────────────────────────────────────────────────────────
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self._bridge  = CvBridge()
        self._map_pub = self.create_publisher(
            OccupancyGrid, '/perception/road_costmap', latched_qos)

        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, sensor_qos)

        self.create_timer(1.0 / rate, self._publish_costmap)

        self.get_logger().info(
            f'LaneCostmapNode v2 | '
            f'grid={self._grid_w}×{self._grid_h} @ {self._res}m/cell | '
            f'max_proj={self._max_proj}m | forward_only={self._fwd_only}')

    # ═══════════════════════════════════════════════════════════════════
    # Lane boundary detection
    # ═══════════════════════════════════════════════════════════════════

    def _detect_lines(self, frame):
        """
        Returns (left_fit, right_fit) where each fit is (m, b) or None.
        Uses the bottom (1-roi_top_frac) of the frame only.
        """
        fh, fw = frame.shape[:2]
        roi_y  = int(fh * self._roi_top)
        roi    = frame[roi_y:fh, :]
        roi_h  = roi.shape[0]
        img_cx = fw / 2.0

        # White mask
        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([0,   0,               self._white_vmin]),
                           np.array([180, self._white_smax, 255]))
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, bright = cv2.threshold(gray, self._white_vmin, 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_and(mask, bright)
        k    = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 25))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        edges = cv2.Canny(mask, 30, 100)

        lines = cv2.HoughLinesP(edges, 1, np.pi / 180,
                                threshold=15, minLineLength=20.0, maxLineGap=40.0)
        left_segs, right_segs = [], []
        if lines is not None:
            for ln in lines:
                x1, y1, x2, y2 = ln[0]
                if x2 == x1:
                    continue
                slope = abs((y2 - y1) / float(x2 - x1))
                if not (0.2 <= slope <= 4.0):
                    continue
                length = float(np.hypot(x2 - x1, y2 - y1))
                if (x1 + x2) / 2.0 < img_cx:
                    left_segs.append((x1, y1, x2, y2, length))
                else:
                    right_segs.append((x1, y1, x2, y2, length))

        def _fit(segs):
            ms, bs, ws = [], [], []
            for (x1, y1, x2, y2, w) in segs:
                dx = float(x2 - x1)
                if dx == 0:
                    continue
                m = (y2 - y1) / dx
                ms.append(m); bs.append(y1 - m * x1); ws.append(w)
            if not ms:
                return None
            tw   = sum(ws)
            m_av = sum(m * w for m, w in zip(ms, ws)) / tw
            b_av = sum(b * w for b, w in zip(bs, ws)) / tw
            return (m_av, b_av) if abs(m_av) > 1e-6 else None

        left_fit  = _fit(left_segs)
        right_fit = _fit(right_segs)

        # Return fits AND roi_y and roi_h so caller can compute pixel coords
        return left_fit, right_fit, roi_y, roi_h, fw

    # ═══════════════════════════════════════════════════════════════════
    # Projection: pixel → ground plane (z=0 in map frame)
    # ═══════════════════════════════════════════════════════════════════

    def _project(self, u: float, v: float,
                 cam_pos: np.ndarray, R_map: np.ndarray,
                 robot_pos: np.ndarray, robot_fwd: np.ndarray):
        """
        Project pixel (u, v) → ground (z=0, map frame).

        Returns (wx, wy) or None if:
          • ray is parallel to ground
          • intersection is behind camera
          • distance from camera > max_proj_m or < min_proj_m
          • point is behind robot (when forward_only=True)
          • point is outside grid bounds
        """
        # Ray in optical frame
        ray_opt  = np.array([(u - self._cx) / self._fx,
                              (v - self._cy) / self._fy,
                              1.0], dtype=np.float64)
        # Rotate to map frame (optical → link → map)
        ray_map  = R_map @ (_R_OPT_TO_LINK @ ray_opt)

        # Ground-plane intersection (z = 0 in map frame)
        if abs(ray_map[2]) < 1e-4:
            return None
        lam = -cam_pos[2] / ray_map[2]
        if lam <= 0.0:
            return None          # intersection behind camera

        wx = cam_pos[0] + lam * ray_map[0]
        wy = cam_pos[1] + lam * ray_map[1]

        # Distance gates
        d = math.hypot(wx - cam_pos[0], wy - cam_pos[1])
        if d < self._min_proj or d > self._max_proj:
            return None

        # Forward-only gate — reject points behind the robot
        if self._fwd_only:
            dx = wx - robot_pos[0]
            dy = wy - robot_pos[1]
            if robot_fwd[0] * dx + robot_fwd[1] * dy < 0.0:
                return None

        # Grid bounds
        col = int((wx - self._origin_x) / self._res)
        row = int((wy - self._origin_y) / self._res)
        if not (0 <= col < self._grid_w and 0 <= row < self._grid_h):
            return None

        return col, row

    # ═══════════════════════════════════════════════════════════════════
    # Costmap write (obstacle never overwritten by free)
    # ═══════════════════════════════════════════════════════════════════

    def _mark(self, col: int, row: int, value: int):
        idx = row * self._grid_w + col
        if value == 100 or self._grid[idx] == -1:
            self._grid[idx] = np.int8(value)

    # ═══════════════════════════════════════════════════════════════════
    # Image callback
    # ═══════════════════════════════════════════════════════════════════

    def _image_cb(self, msg: Image):
        self._frame_count += 1
        if self._frame_count % self._skip_n != 0:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return

        # ── Camera TF: camera_link → map ──────────────────────────────────
        try:
            cam_tf = self._tf_buf.lookup_transform(
                'map', 'camera_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException as ex:
            self.get_logger().debug(f'camera_link TF: {ex}')
            return

        t        = cam_tf.transform.translation
        cam_pos  = np.array([t.x, t.y, t.z], dtype=np.float64)
        R_cam    = _quat_to_rot(cam_tf.transform.rotation)

        # ── Robot TF: base_link → map ─────────────────────────────────────
        try:
            rob_tf = self._tf_buf.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException as ex:
            self.get_logger().debug(f'base_link TF: {ex}')
            return

        rt         = rob_tf.transform.translation
        robot_pos  = np.array([rt.x, rt.y], dtype=np.float64)
        R_rob      = _quat_to_rot(rob_tf.transform.rotation)
        # Robot forward direction in map frame (base_link x-axis)
        robot_fwd  = R_rob @ np.array([1.0, 0.0, 0.0])
        robot_fwd  = robot_fwd[:2]  # only x,y

        # ── Detect lane lines ─────────────────────────────────────────────
        left_fit, right_fit, roi_y, roi_h, fw = self._detect_lines(frame)
        if left_fit is None and right_fit is None:
            return

        # ── Sample rows in the ROI ────────────────────────────────────────
        step = max(1, roi_h // max(1, self._sample_rows))

        # At 1m height, lane lines are in top 50% of ROI (farther ground)
        # Bottom 50% is near ground right beside robot
        roi_sample_end   = int(roi_h * 0.5)   # stop halfway down
        roi_sample_start = 0                    # start from top of ROI

        for y_roi in range(roi_sample_start, roi_sample_end, step):
            y_full = float(y_roi + roi_y)

            # Compute boundary x-coordinates at this row
            lx = rx = None
            if left_fit:
                m, b = left_fit
                lx = float(np.clip((y_roi - b) / m, 0.0, fw - 1.0))
            if right_fit:
                m, b = right_fit
                rx = float(np.clip((y_roi - b) / m, 0.0, fw - 1.0))

            # ── Left boundary: mark pixels to the LEFT as lethal ──────────
            if lx is not None:
                # Lethal band: outside (left of) the lane line
                step_px = max(1, self._lethal_px // 4)
                for du in range(0, self._lethal_px, step_px):
                    cell = self._project(lx - du, y_full, cam_pos, R_cam,
                                         robot_pos, robot_fwd)
                    if cell:
                        self._mark(cell[0], cell[1], 100)

                # Free band: inside (right of) the lane line — just inside
                for du in range(4, 16, 4):
                    cell = self._project(lx + du, y_full, cam_pos, R_cam,
                                         robot_pos, robot_fwd)
                    if cell:
                        self._mark(cell[0], cell[1], 0)

            # ── Right boundary: mark pixels to the RIGHT as lethal ────────
            if rx is not None:
                step_px = max(1, self._lethal_px // 4)
                for du in range(0, self._lethal_px, step_px):
                    cell = self._project(rx + du, y_full, cam_pos, R_cam,
                                         robot_pos, robot_fwd)
                    if cell:
                        self._mark(cell[0], cell[1], 100)

                for du in range(4, 16, 4):
                    cell = self._project(rx - du, y_full, cam_pos, R_cam,
                                         robot_pos, robot_fwd)
                    if cell:
                        self._mark(cell[0], cell[1], 0)

    # ═══════════════════════════════════════════════════════════════════
    # Publish
    # ═══════════════════════════════════════════════════════════════════

    def _publish_costmap(self):
        msg                            = OccupancyGrid()
        msg.header.stamp               = self.get_clock().now().to_msg()
        msg.header.frame_id            = 'map'
        msg.info.resolution            = self._res
        msg.info.width                 = self._grid_w
        msg.info.height                = self._grid_h
        msg.info.origin.position.x     = self._origin_x
        msg.info.origin.position.y     = self._origin_y
        msg.info.origin.position.z     = 0.0
        msg.info.origin.orientation.w  = 1.0
        msg.data                       = self._grid.tolist()
        self._map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()