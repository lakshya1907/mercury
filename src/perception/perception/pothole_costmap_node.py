"""
pothole_costmap_node.py  (v1)
------------------------------
Detects pothole blobs from the camera image, projects them onto the
ground plane, and publishes a PERSISTENT OccupancyGrid on
/perception/pothole_costmap (map frame, latched QoS).

WHY GLOBAL COSTMAP:
  Potholes don't move. Once detected and projected to map coordinates,
  they must persist even after the pothole leaves the camera FOV.
  The global costmap (map frame, persistent) is the right place.
  Nav2's global planner + DWB local planner both read it → robot
  avoids potholes from far away AND up close.

DETECTION LOGIC (inverse of lane_costmap _filter_blobs):
  A blob is a POTHOLE when:
    1. area          >= blob_min_area      (not speckle noise)
    2. circularity   >= blob_min_circ      (round/oval shape)   ← key test
    3. aspect ratio  <  blob_max_aspect    (not an elongated stripe)

  circularity = 4π × area / perimeter²
    • Pothole / oval  → 0.45 – 1.0   (detected)
    • Lane stripe     → 0.05 – 0.25  (ignored)

RADIUS ESTIMATION (automatic, size-agnostic):
  For each detected pothole blob:
    1. Sample N points evenly around the contour
    2. Project each point to the ground plane via ray-casting + TF
    3. Take the median distance from the projected centroid as r_detected
    4. Clamp to [min_pothole_r, max_pothole_r] for safety
    5. Add inflation_pad on top
  This adapts automatically to any pothole size — no hardcoded dimensions.

PERSISTENCE:
  No decay. Cells written once stay for the entire run.
  Rationale: potholes are static. If the robot re-visits an area,
  the costmap correctly still shows the pothole.
  To reset: restart the node.
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


class PotholeCostmapNode(Node):

    def __init__(self):
        super().__init__('pothole_costmap')

        # ── Parameters ───────────────────────────────────────────────────────
        # Map extent — MUST match global_costmap.yaml
        self.declare_parameter('map_width_m',   70.0)
        self.declare_parameter('map_height_m',  70.0)
        self.declare_parameter('resolution',     0.10)
        self.declare_parameter('map_origin_x', -35.0)
        self.declare_parameter('map_origin_y', -35.0)
        self.declare_parameter('publish_rate',   2.0)   # Hz — slower OK, persistent

        # Camera intrinsics
        self.declare_parameter('camera_hfov',   1.047)
        self.declare_parameter('image_width',   640)
        self.declare_parameter('image_height',  480)
        self.declare_parameter('roi_top_frac',  0.35)   # ignore sky

        # Projection safety gates
        self.declare_parameter('max_proj_m',    5.0)    # ignore far detections
        self.declare_parameter('min_proj_m',    0.3)    # ignore too-close noise
        self.declare_parameter('forward_only',  True)

        # White pixel thresholds (potholes are bright white in sim)
        self.declare_parameter('white_v_min',   130)
        self.declare_parameter('white_s_max',    80)

        # ── Pothole blob criteria ─────────────────────────────────────────────
        # Minimum blob area — ignore speckle / tiny noise
        self.declare_parameter('blob_min_area',      400)

        # Circularity threshold — blobs ABOVE this are treated as potholes.
        # Potholes: ≥ 0.45.  Lane stripes: ≤ 0.25.  Use 0.40 for margin.
        self.declare_parameter('blob_min_circularity', 0.40)

        # Aspect ratio (minAreaRect long/short) — blobs BELOW this are round.
        # Lane stripes: ≥ 2.5.  Potholes: ≤ 2.0.  Use 2.2 for margin.
        self.declare_parameter('blob_max_aspect',      2.2)

        # ── Pothole size in world coordinates ─────────────────────────────────
        # Radius is estimated from the projected contour automatically.
        # These clamps handle degenerate projections (grazing angle, far away).
        self.declare_parameter('min_pothole_r',   0.20)  # metres
        self.declare_parameter('max_pothole_r',   1.20)  # metres

        # Extra padding added around the estimated radius before marking lethal.
        # Acts as a safety margin — Nav2 inflation layer adds more on top.
        self.declare_parameter('inflation_pad',   0.15)  # metres

        # How many contour points to sample for radius estimation
        self.declare_parameter('radius_samples',  12)

        # Skip every N frames (CPU saving — potholes are static)
        self.declare_parameter('process_every_n', 5)

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
        self._blob_min_area  = int(_p('blob_min_area'))
        self._blob_min_circ  = float(_p('blob_min_circularity'))
        self._blob_max_asp   = float(_p('blob_max_aspect'))
        self._min_r          = float(_p('min_pothole_r'))
        self._max_r          = float(_p('max_pothole_r'))
        self._inflation_pad  = float(_p('inflation_pad'))
        self._radius_samples = int(_p('radius_samples'))
        self._skip_n         = int(_p('process_every_n'))

        # ── Derived ──────────────────────────────────────────────────────────
        self._grid_w = int(round(map_w_m  / self._res))
        self._grid_h = int(round(map_h_m  / self._res))
        self._fx     = (self._img_w / 2.0) / math.tan(self._hfov / 2.0)
        self._fy     = self._fx
        self._cx     = self._img_w / 2.0
        self._cy     = self._img_h / 2.0

        # Persistent grid — never decays
        self._grid = np.full(self._grid_w * self._grid_h, -1, dtype=np.int8)

        # Track which map cells have been marked — for logging
        self._pothole_count = 0
        self._frame_count   = 0

        # ── TF ───────────────────────────────────────────────────────────────
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        # ── QoS ──────────────────────────────────────────────────────────────
        # Latched — new subscribers receive the latest costmap immediately
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
            OccupancyGrid, '/perception/pothole_costmap', latched_qos)

        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, sensor_qos)

        self.create_timer(1.0 / rate, self._publish_costmap)

        self.get_logger().info(
            f'PotholeCostmapNode v1 | '
            f'grid={self._grid_w}×{self._grid_h} @ {self._res}m/cell | '
            f'blob_min_circ={self._blob_min_circ} | '
            f'blob_max_aspect={self._blob_max_asp} | '
            f'r=[{self._min_r},{self._max_r}]m + pad={self._inflation_pad}m | '
            f'PERSISTENT (no decay)')

    # ═══════════════════════════════════════════════════════════════════
    # Pothole blob detection
    # ═══════════════════════════════════════════════════════════════════

    def _detect_potholes(self, frame) -> list[dict]:
        """
        Detect pothole blobs in the camera image.

        Returns list of dicts, each with:
          'centroid_uv' : (u, v) pixel centroid in FULL frame coords
          'contour_uvs' : list of (u, v) sampled from contour, full frame coords
          'area_px'     : blob area in pixels
          'circularity' : blob circularity
          'aspect'      : minAreaRect aspect ratio
        """
        fh, fw = frame.shape[:2]
        roi_y  = int(fh * self._roi_top)
        roi    = frame[roi_y:fh, :]

        # ── White pixel mask ─────────────────────────────────────────────────
        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([0,   0,               self._white_vmin]),
                           np.array([180, self._white_smax, 255]))
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, bright = cv2.threshold(
            gray, self._white_vmin, 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_and(mask, bright)

        # Mild morphological close — fills small holes inside pothole blob
        k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        # ── Find contours ─────────────────────────────────────────────────────
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        potholes = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self._blob_min_area:
                continue

            perimeter = cv2.arcLength(cnt, closed=True)
            if perimeter < 1.0:
                continue

            circularity = (4.0 * math.pi * area) / (perimeter * perimeter)

            rect       = cv2.minAreaRect(cnt)
            w, h       = rect[1]
            if w < 1.0 or h < 1.0:
                continue
            aspect = max(w, h) / min(w, h)

            # ── Pothole criteria: round AND not elongated ─────────────────────
            if circularity < self._blob_min_circ:
                continue   # too elongated — lane stripe
            if aspect >= self._blob_max_asp:
                continue   # too elongated — lane stripe

            # ── Compute centroid in full-frame coords ─────────────────────────
            M  = cv2.moments(cnt)
            if M['m00'] < 1.0:
                continue
            cu = M['m10'] / M['m00']
            cv_coord = M['m01'] / M['m00'] + roi_y   # offset back to full frame

            # ── Sample contour points evenly ──────────────────────────────────
            n_pts    = len(cnt)
            step     = max(1, n_pts // self._radius_samples)
            sampled  = cnt[::step]
            contour_uvs = [
                (float(pt[0][0]), float(pt[0][1]) + roi_y)
                for pt in sampled
            ]

            potholes.append({
                'centroid_uv': (cu, cv_coord),
                'contour_uvs': contour_uvs,
                'area_px':     area,
                'circularity': circularity,
                'aspect':      aspect,
            })

            self.get_logger().debug(
                f'[Pothole] blob: area={area:.0f}px² '
                f'circ={circularity:.3f} aspect={aspect:.2f}')

        return potholes

    # ═══════════════════════════════════════════════════════════════════
    # Ray-cast: pixel (u,v) → world ground point (wx, wy) or None
    # ═══════════════════════════════════════════════════════════════════

    def _pixel_to_ground(self, u, v, cam_pos, R_cam, robot_pos, robot_fwd):
        ray_opt = np.array([(u  - self._cx) / self._fx,
                             (v  - self._cy) / self._fy,
                             1.0], dtype=np.float64)
        ray_map = R_cam @ (_R_OPT_TO_LINK @ ray_opt)

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
            if (robot_fwd[0] * (wx - robot_pos[0]) +
                    robot_fwd[1] * (wy - robot_pos[1])) < 0.0:
                return None

        return wx, wy

    # ═══════════════════════════════════════════════════════════════════
    # Mark a filled circle of lethal cells on the persistent grid
    # ═══════════════════════════════════════════════════════════════════

    def _mark_circle(self, cx_m, cy_m, radius_m):
        """Mark all grid cells within radius_m of (cx_m, cy_m) as lethal."""
        r_cells = int(math.ceil(radius_m / self._res))

        cx_cell = int((cx_m - self._origin_x) / self._res)
        cy_cell = int((cy_m - self._origin_y) / self._res)

        marked = 0
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                if dr * dr + dc * dc > r_cells * r_cells:
                    continue
                col = cx_cell + dc
                row = cy_cell + dr
                if 0 <= col < self._grid_w and 0 <= row < self._grid_h:
                    self._grid[row * self._grid_w + col] = np.int8(100)
                    marked += 1

        return marked

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

        # ── TF lookups ────────────────────────────────────────────────────────
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

        # ── Detect potholes ───────────────────────────────────────────────────
        potholes = self._detect_potholes(frame)
        if not potholes:
            return

        for ph in potholes:
            # Project centroid to ground
            cu, cv_coord = ph['centroid_uv']
            center_gnd = self._pixel_to_ground(
                cu, cv_coord, cam_pos, R_cam, robot_pos, robot_fwd)
            if center_gnd is None:
                continue

            cx_m, cy_m = center_gnd

            # ── Estimate radius from contour projections ──────────────────────
            projected_dists = []
            for (u, v) in ph['contour_uvs']:
                gnd = self._pixel_to_ground(
                    u, v, cam_pos, R_cam, robot_pos, robot_fwd)
                if gnd is None:
                    continue
                d = math.hypot(gnd[0] - cx_m, gnd[1] - cy_m)
                projected_dists.append(d)

            if projected_dists:
                # Median is robust to outlier projections (grazing rays)
                r_estimated = float(np.median(projected_dists))
            else:
                # Fallback: can't estimate — use minimum safe radius
                r_estimated = self._min_r

            # Clamp + add inflation pad
            r_final = float(np.clip(r_estimated, self._min_r, self._max_r))
            r_final += self._inflation_pad

            # ── Skip if already marked ────────────────────────────────────────
            cx_cell = int((cx_m - self._origin_x) / self._res)
            cy_cell = int((cy_m - self._origin_y) / self._res)
            if (0 <= cx_cell < self._grid_w and 0 <= cy_cell < self._grid_h
                    and self._grid[cy_cell * self._grid_w + cx_cell] == 100):
                continue

            # ── Mark on persistent grid ───────────────────────────────────────
            cells_marked = self._mark_circle(cx_m, cy_m, r_final)
            self._pothole_count += 1

            self.get_logger().info(
                f'[Pothole] #{self._pothole_count} detected → '
                f'map=({cx_m:.2f}, {cy_m:.2f})  '
                f'r_est={r_estimated:.2f}m  r_final={r_final:.2f}m  '
                f'cells={cells_marked}  '
                f'circ={ph["circularity"]:.3f}  aspect={ph["aspect"]:.2f}')

    # ═══════════════════════════════════════════════════════════════════
    # Publish persistent costmap
    # ═══════════════════════════════════════════════════════════════════

    def _publish_costmap(self):
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
    node = PotholeCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()