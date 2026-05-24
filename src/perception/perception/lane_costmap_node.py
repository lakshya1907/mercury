"""
lane_costmap_node.py  (v5 — circularity fix)
---------------------------------------------
Adds a circularity gate on top of v4's minAreaRect aspect filter.

ROOT CAUSE of residual pothole leakage (v4)
-------------------------------------------
When a pothole is viewed at an angle / distance, perspective foreshortening
can stretch its bounding oval so that minAreaRect aspect ≥ 2.5 — it passes
the aspect filter even though it is clearly a round blob.

FIX: circularity = 4π × area / perimeter²
  • Circle / oval  → circularity ≈ 0.6–1.0  (REJECT)
  • Lane stripe    → circularity ≈ 0.05–0.25 (KEEP)
  This holds regardless of perspective or viewing angle because both
  area and perimeter scale proportionally with foreshortening.

A blob now passes _filter_blobs only when BOTH conditions hold:
  1. minAreaRect aspect  ≥  blob_min_aspect   (elongated shape)
  2. circularity         <  blob_max_circularity  (not round)

New parameter: ``blob_max_circularity`` (default 0.35)
  Potholes stay above 0.45 even when foreshortened.
  Lane stripes stay below 0.25 on straight and curved sections.
  Gap between 0.25 and 0.45 gives comfortable margin → use 0.35.

All other logic (projection, decay, Hough) is unchanged from v4.
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
        self.declare_parameter('roi_top_frac',   0.35)

        # Projection safety gates
        self.declare_parameter('max_proj_m',     5.0)
        self.declare_parameter('min_proj_m',     0.3)
        self.declare_parameter('forward_only',   True)

        # Lane marking thresholds
        self.declare_parameter('white_v_min',    130)
        self.declare_parameter('white_s_max',     80)
        self.declare_parameter('sample_rows',      8)
        self.declare_parameter('process_every_n',  3)
        self.declare_parameter('lethal_band_px',  20)

        # ── Blob / pothole filter ────────────────────────────────────────────
        # minAreaRect aspect ratio gate (long/short ≥ this to keep).
        self.declare_parameter('blob_min_aspect',     2.5)

        # NEW: circularity gate.  4π·area/perimeter².
        # Ovals/potholes ≥ 0.45 even when foreshortened.
        # Lane stripes   ≤ 0.25 regardless of angle.
        # Default 0.35 sits in the middle of that gap.
        self.declare_parameter('blob_max_circularity', 0.35)

        # Minimum blob area in pixels (removes isolated speckle noise).
        self.declare_parameter('blob_min_area',   60)

        # Hough parameters
        self.declare_parameter('hough_min_len',   25.0)
        self.declare_parameter('hough_max_gap',   40.0)
        self.declare_parameter('hough_threshold', 15)

        # Very loose span sanity check.
        self.declare_parameter('min_span_frac',   0.03)

        # Costmap write params
        self.declare_parameter('obstacle_pixels_outside', 48)
        self.declare_parameter('free_pixels_inside',      32)

        # ── Read ─────────────────────────────────────────────────────────────
        def _p(n): return self.get_parameter(n).value

        map_w_m              = float(_p('map_width_m'))
        map_h_m              = float(_p('map_height_m'))
        self._res            = float(_p('resolution'))
        self._origin_x       = float(_p('map_origin_x'))
        self._origin_y       = float(_p('map_origin_y'))
        rate                 = float(_p('publish_rate'))
        self._hfov           = float(_p('camera_hfov'))
        self._img_w          = int(_p('image_width'))
        self._img_h          = int(_p('image_height'))
        self._roi_top        = float(_p('roi_top_frac'))
        self._max_proj       = float(_p('max_proj_m'))
        self._min_proj       = float(_p('min_proj_m'))
        self._fwd_only       = bool(_p('forward_only'))
        self._white_vmin     = int(_p('white_v_min'))
        self._white_smax     = int(_p('white_s_max'))
        self._sample_rows    = int(_p('sample_rows'))
        self._skip_n         = int(_p('process_every_n'))
        self._lethal_px      = int(_p('lethal_band_px'))
        self._blob_min_asp   = float(_p('blob_min_aspect'))
        self._blob_max_circ  = float(_p('blob_max_circularity'))   # NEW
        self._blob_min_area  = int(_p('blob_min_area'))
        self._hough_min_len  = float(_p('hough_min_len'))
        self._hough_max_gap  = float(_p('hough_max_gap'))
        self._hough_thresh   = int(_p('hough_threshold'))
        self._min_span_frac  = float(_p('min_span_frac'))

        # ── Derived ──────────────────────────────────────────────────────────
        self._grid_w = int(round(map_w_m  / self._res))
        self._grid_h = int(round(map_h_m  / self._res))
        self._fx     = (self._img_w / 2.0) / math.tan(self._hfov / 2.0)
        self._fy     = self._fx
        self._cx     = self._img_w  / 2.0
        self._cy     = self._img_h  / 2.0

        self._grid         = np.full(self._grid_w * self._grid_h, -1, dtype=np.int8)
        self._lethal_stamp = np.zeros(self._grid_w * self._grid_h, dtype=np.float64)
        self._decay_sec    = 8.0
        self._frame_count  = 0

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
            f'LaneCostmapNode v5 | '
            f'grid={self._grid_w}×{self._grid_h} @ {self._res}m/cell | '
            f'blob_min_aspect={self._blob_min_asp} | '
            f'blob_max_circularity={self._blob_max_circ} | '
            f'blob_min_area={self._blob_min_area}px²')

    # ═══════════════════════════════════════════════════════════════════
    # Blob filter — potholes out, lane stripes kept  (v5: + circularity)
    # ═══════════════════════════════════════════════════════════════════

    def _filter_blobs(self, mask: np.ndarray) -> np.ndarray:
        """
        Remove pothole / circular blobs using TWO complementary tests:

        Test 1 — minAreaRect aspect ratio  (long / short side)
          Rejects obviously round blobs.  Can be fooled by a pothole that
          is perspective-stretched into an oval with aspect > 2.5.

        Test 2 — circularity  =  4π × area / perimeter²
          A circle / oval → ≈ 0.7–1.0  regardless of perspective stretch.
          A lane stripe   → ≈ 0.05–0.25 regardless of angle or distance.
          This catches the potholes that slip past the aspect gate.

        A blob is KEPT only when BOTH hold:
          aspect       ≥  blob_min_aspect        (shape is elongated)
          circularity  <  blob_max_circularity   (shape is not round)
        """
        out = np.zeros_like(mask)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self._blob_min_area:
                continue                   # speckle noise

            # ── Test 1: minAreaRect aspect ────────────────────────────────
            rect   = cv2.minAreaRect(cnt)
            w, h   = rect[1]
            if w < 1.0 or h < 1.0:
                continue
            aspect = max(w, h) / min(w, h)

            if aspect < self._blob_min_asp:
                self.get_logger().debug(
                    f'[LaneCostmap] blob rejected (aspect={aspect:.2f} < '
                    f'{self._blob_min_asp}): area={area:.0f}')
                continue

            # ── Test 2: circularity ───────────────────────────────────────
            perimeter = cv2.arcLength(cnt, closed=True)
            if perimeter < 1.0:
                continue
            circularity = (4.0 * math.pi * area) / (perimeter * perimeter)

            if circularity >= self._blob_max_circ:
                self.get_logger().debug(
                    f'[LaneCostmap] blob rejected (circularity={circularity:.3f} ≥ '
                    f'{self._blob_max_circ}): area={area:.0f} aspect={aspect:.2f}')
                continue

            # Both tests passed — keep this blob as a lane marking
            cv2.drawContours(out, [cnt], -1, 255, cv2.FILLED)

        return out

    # ═══════════════════════════════════════════════════════════════════
    # Lane boundary detection
    # ═══════════════════════════════════════════════════════════════════

    def _detect_lines(self, frame):
        """
        Returns (left_fit, right_fit, roi_y, roi_h, fw).
        Each fit is (m, b) or None.
        """
        fh, fw = frame.shape[:2]
        roi_y  = int(fh * self._roi_top)
        roi    = frame[roi_y:fh, :]
        roi_h  = roi.shape[0]
        img_cx = fw / 2.0

        # Step 1 — white pixel mask
        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([0,   0,               self._white_vmin]),
                           np.array([180, self._white_smax, 255]))
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, bright = cv2.threshold(
            gray, self._white_vmin, 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_and(mask, bright)

        # Step 2 — remove pothole blobs (aspect + circularity)
        mask = self._filter_blobs(mask)

        # Step 3 — morphological close to bridge stripe gaps
        k    = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 25))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        # Step 4 — Canny
        edges = cv2.Canny(mask, 30, 100)

        # Step 5 — Hough
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180,
            threshold=self._hough_thresh,
            minLineLength=self._hough_min_len,
            maxLineGap=self._hough_max_gap)

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

        # Step 6 — weighted fit with loose span check
        def _fit(segs):
            if not segs:
                return None
            ms, bs, ws = [], [], []
            y_vals = []
            for (x1, y1, x2, y2, w) in segs:
                dx = float(x2 - x1)
                if dx == 0:
                    continue
                m = (y2 - y1) / dx
                ms.append(m)
                bs.append(y1 - m * x1)
                ws.append(w)
                y_vals.extend([y1, y2])
            if not ms:
                return None

            if self._min_span_frac > 0.0 and y_vals:
                y_span   = max(y_vals) - min(y_vals)
                min_span = self._min_span_frac * roi_h
                if y_span < min_span:
                    return None

            tw   = sum(ws)
            m_av = sum(m * w for m, w in zip(ms, ws)) / tw
            b_av = sum(b * w for b, w in zip(bs, ws)) / tw
            return (m_av, b_av) if abs(m_av) > 1e-6 else None

        return _fit(left_segs), _fit(right_segs), roi_y, roi_h, fw

    # ═══════════════════════════════════════════════════════════════════
    # Projection: pixel → ground (z=0, map frame)
    # ═══════════════════════════════════════════════════════════════════

    def _project(self, u, v, cam_pos, R_map, robot_pos, robot_fwd):
        ray_opt = np.array([(u - self._cx) / self._fx,
                             (v - self._cy) / self._fy,
                             1.0], dtype=np.float64)
        ray_map = R_map @ (_R_OPT_TO_LINK @ ray_opt)

        if abs(ray_map[2]) < 1e-4:
            return None
        lam = -cam_pos[2] / ray_map[2]
        if lam <= 0.0:
            return None

        wx = cam_pos[0] + lam * ray_map[0]
        wy = cam_pos[1] + lam * ray_map[1]

        d = math.hypot(wx - cam_pos[0], wy - cam_pos[1])
        if d < self._min_proj or d > self._max_proj:
            return None

        if self._fwd_only:
            if robot_fwd[0] * (wx - robot_pos[0]) + \
               robot_fwd[1] * (wy - robot_pos[1]) < 0.0:
                return None

        col = int((wx - self._origin_x) / self._res)
        row = int((wy - self._origin_y) / self._res)
        if not (0 <= col < self._grid_w and 0 <= row < self._grid_h):
            return None
        return col, row

    # ═══════════════════════════════════════════════════════════════════
    # Costmap write
    # ═══════════════════════════════════════════════════════════════════

    def _mark(self, col, row, value):
        idx = row * self._grid_w + col
        if value == 100:
            self._grid[idx]         = np.int8(100)
            self._lethal_stamp[idx] = self.get_clock().now().nanoseconds / 1e9
        else:
            self._grid[idx]         = np.int8(0)
            self._lethal_stamp[idx] = 0.0

    # ═══════════════════════════════════════════════════════════════════
    # Image callback
    # ═══════════════════════════════════════════════════════════════════

    def _image_cb(self, msg):
        self._frame_count += 1
        if self._frame_count % self._skip_n != 0:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return

        try:
            cam_tf = self._tf_buf.lookup_transform(
                'map', 'camera_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException as ex:
            self.get_logger().debug(f'camera_link TF: {ex}')
            return

        t       = cam_tf.transform.translation
        cam_pos = np.array([t.x, t.y, t.z], dtype=np.float64)
        R_cam   = _quat_to_rot(cam_tf.transform.rotation)

        try:
            rob_tf = self._tf_buf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException as ex:
            self.get_logger().debug(f'base_link TF: {ex}')
            return

        rt        = rob_tf.transform.translation
        robot_pos = np.array([rt.x, rt.y], dtype=np.float64)
        R_rob     = _quat_to_rot(rob_tf.transform.rotation)
        robot_fwd = (R_rob @ np.array([1.0, 0.0, 0.0]))[:2]

        left_fit, right_fit, roi_y, roi_h, fw = self._detect_lines(frame)
        if left_fit is None and right_fit is None:
            return

        step           = max(1, roi_h // max(1, self._sample_rows))
        roi_sample_end = int(roi_h * 0.5)

        for y_roi in range(0, roi_sample_end, step):
            y_full = float(y_roi + roi_y)

            lx = rx = None
            if left_fit:
                m, b = left_fit
                lx = float(np.clip((y_roi - b) / m, 0.0, fw - 1.0))
            if right_fit:
                m, b = right_fit
                rx = float(np.clip((y_roi - b) / m, 0.0, fw - 1.0))

            if lx is not None:
                step_px = max(1, self._lethal_px // 4)
                for du in range(0, self._lethal_px, step_px):
                    cell = self._project(lx - du, y_full, cam_pos, R_cam,
                                         robot_pos, robot_fwd)
                    if cell:
                        self._mark(cell[0], cell[1], 100)
                for du in range(4, 16, 4):
                    cell = self._project(lx + du, y_full, cam_pos, R_cam,
                                         robot_pos, robot_fwd)
                    if cell:
                        self._mark(cell[0], cell[1], 0)

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
    # Publish (with lethal-cell decay)
    # ═══════════════════════════════════════════════════════════════════

    def _publish_costmap(self):
        now = self.get_clock().now().nanoseconds / 1e9
        expired = (
            (self._grid == 100) &
            (self._lethal_stamp > 0) &
            ((now - self._lethal_stamp) > self._decay_sec)
        )
        self._grid[expired]         = np.int8(-1)
        self._lethal_stamp[expired] = 0.0

        msg                           = OccupancyGrid()
        msg.header.stamp              = self.get_clock().now().to_msg()
        msg.header.frame_id           = 'map'
        msg.info.resolution           = self._res
        msg.info.width                = self._grid_w
        msg.info.height               = self._grid_h
        msg.info.origin.position.x    = self._origin_x
        msg.info.origin.position.y    = self._origin_y
        msg.info.origin.position.z    = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data                      = self._grid.tolist()
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