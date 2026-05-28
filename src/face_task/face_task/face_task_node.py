#!/usr/bin/env python3
"""
face_task_node.py
=================
Brain node for the face recognition task.

State machine:
    IDLE        -- waiting for /face_task/start
    SCANNING    -- iterating through 21 (H×V) turret positions, capturing + recognising
    FINE_TUNE   -- proportional controller to centre face in frame
    FIRE        -- activate laser, wait, publish done
    DONE        -- publish /face_task/complete, return to IDLE

Joint / angle convention (matches URDF and fixed turret_gazebo_bridge):
    pan_deg  :  0 = straight forward (+X)
                positive = LEFT  (CCW from above, matches Z-up axis)
                range    : [-170, +170] deg
    tilt_deg :  0 = level (horizontal)
                positive = UP
                negative = DOWN
                range    : [-80, +80] deg

Scan grid  (7 pan × 3 tilt = 21 positions):
    pan  (deg): [-120, -80, -40, 0, 40, 80, 120]   — covers ±120° sweep
    tilt (deg): [+20, 0, -20]                        — high / level / low

    The face photos in mercury.sdf are at z=1.5m, robot base ~z=0.21m,
    turret ~z=0.26m above base → photos are ~1.24m above turret at ~50m
    distance → elevation angle ≈ atan(1.24/50) ≈ 1.4°.
    So tilt=0 (level) is the critical row.  ±20° gives comfortable margin.

Subscribes:
    /face_task/start           (std_msgs/Bool)   -- True starts the task
    /face/match_found          (std_msgs/Bool)
    /face/horizontal_error     (std_msgs/Float32) -- pixels, + = face right of centre
    /face/vertical_error       (std_msgs/Float32) -- pixels, + = face below centre

Publishes:
    /face/capture_request      (std_msgs/Bool)
    /turret/pan_deg            (std_msgs/Float32)  -- degrees, 0=forward, +=left
    /turret/tilt_deg           (std_msgs/Float32)  -- degrees, 0=level,  +=up
    /laser/fire                (std_msgs/Bool)
    /face_task/complete        (std_msgs/Bool)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import time

# ── State constants ────────────────────────────────────────────────────────────
IDLE       = 'IDLE'
SCANNING   = 'SCANNING'
FINE_TUNE  = 'FINE_TUNE'
FIRE       = 'FIRE'
DONE       = 'DONE'


class FaceTaskNode(Node):

    def __init__(self):
        super().__init__('face_task_node')

        # ── Parameters ────────────────────────────────────────────────────────
        # Pan angles: 0=forward, sweep ±120° in 40° steps
        self.declare_parameter('h_positions_deg',
                               [-120.0, -80.0, -40.0, 0.0, 40.0, 80.0, 120.0])
        # Tilt angles: 0=level; photos at ~50m are nearly level so centre row is key
        self.declare_parameter('v_positions_deg',  [20.0, 0.0, -20.0])
        self.declare_parameter('settle_time_sec',   0.6)
        self.declare_parameter('fine_tune_px_tol',  20.0)
        # deg/px gain: image ~640px wide, FOV ~60° → ~0.094°/px; 0.05 is conservative
        self.declare_parameter('fine_tune_gain_h',  0.05)
        self.declare_parameter('fine_tune_gain_v',  0.05)
        self.declare_parameter('fine_tune_timeout', 5.0)
        self.declare_parameter('laser_on_time_sec', 3.0)

        self._h_pos      = list(self.get_parameter('h_positions_deg').value)
        self._v_pos      = list(self.get_parameter('v_positions_deg').value)
        self._settle     = self.get_parameter('settle_time_sec').value
        self._tol        = self.get_parameter('fine_tune_px_tol').value
        self._gain_h     = self.get_parameter('fine_tune_gain_h').value
        self._gain_v     = self.get_parameter('fine_tune_gain_v').value
        self._ft_timeout = self.get_parameter('fine_tune_timeout').value
        self._laser_time = self.get_parameter('laser_on_time_sec').value

        # ── Build 21-position scan grid (boustrophedon / snake pattern) ───────
        # Row 0 (tilt high):  pan L→R
        # Row 1 (tilt level): pan R→L
        # Row 2 (tilt low):   pan L→R
        self._grid = []
        for row_idx, tilt in enumerate(self._v_pos):
            pan_row = self._h_pos if row_idx % 2 == 0 else list(reversed(self._h_pos))
            for pan in pan_row:
                self._grid.append((pan, tilt))

        self.get_logger().info(
            f'Scan grid: {len(self._grid)} positions | '
            f'pan={self._h_pos} | tilt={self._v_pos}')
        self.get_logger().info(
            'Convention: pan 0=forward +=left, tilt 0=level +=up')

        # ── State ─────────────────────────────────────────────────────────────
        self._state          = IDLE
        self._grid_idx       = 0
        self._waiting_result = False
        self._match_found    = False
        self._h_err          = 0.0
        self._v_err          = 0.0
        self._ft_start       = None
        self._fire_start     = None

        # Track current turret position for fine-tune adjustments
        self._cur_pan  = 0.0
        self._cur_tilt = 0.0

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Bool,    '/face_task/start',       self._start_cb, 10)
        self.create_subscription(Bool,    '/face/match_found',      self._match_cb, 10)
        self.create_subscription(Float32, '/face/horizontal_error', self._herr_cb,  10)
        self.create_subscription(Float32, '/face/vertical_error',   self._verr_cb,  10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_capture  = self.create_publisher(Bool,    '/face/capture_request', 10)
        self._pub_pan      = self.create_publisher(Float32, '/turret/pan_deg',       10)
        self._pub_tilt     = self.create_publisher(Float32, '/turret/tilt_deg',      10)
        self._pub_laser    = self.create_publisher(Bool,    '/laser/fire',           10)
        self._pub_complete = self.create_publisher(Bool,    '/face_task/complete',   10)

        # Park turret at home on start (forward, level)
        self._move_turret(0.0, 0.0)

        # ── Main loop at 10 Hz ────────────────────────────────────────────────
        self._loop_timer = self.create_timer(0.1, self._loop)
        self.get_logger().info('FaceTaskNode ready. Waiting for /face_task/start ...')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _start_cb(self, msg: Bool):
        if msg.data and self._state == IDLE:
            self.get_logger().info('Task START received. Beginning grid scan.')
            self._reset_scan()
            self._state = SCANNING

    def _match_cb(self, msg: Bool):
        if self._waiting_result:
            self._match_found    = msg.data
            self._waiting_result = False

    def _herr_cb(self, msg: Float32):
        self._h_err = msg.data

    def _verr_cb(self, msg: Float32):
        self._v_err = msg.data

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        if self._state == IDLE:
            return
        elif self._state == SCANNING:
            self._run_scanning()
        elif self._state == FINE_TUNE:
            self._run_fine_tune()
        elif self._state == FIRE:
            self._run_fire()
        # DONE: do nothing

    # ── SCANNING ──────────────────────────────────────────────────────────────

    def _run_scanning(self):
        # Timeout guard: if recognition node doesn't reply, move on
        if self._waiting_result:
            capture_timeout = self._settle + 1.0
            if hasattr(self, '_capture_sent_t') and \
                    time.time() - self._capture_sent_t > capture_timeout:
                self.get_logger().warn(
                    f'[{self._grid_idx+1:02d}/{len(self._grid)}] No response — '
                    f'treating as no-match and advancing.')
                self._waiting_result = False
                self._match_found    = False
            else:
                return

        if self._grid_idx >= len(self._grid):
            self.get_logger().warn(
                'Scan complete — target NOT found. Publishing complete (no match).')
            self._pub_complete.publish(Bool(data=False))
            self._state = DONE
            return

        pan_deg, tilt_deg = self._grid[self._grid_idx]

        if not hasattr(self, '_scan_substep'):
            self._scan_substep = 0
            self._scan_step_t  = None

        if self._scan_substep == 0:
            self._move_turret(pan_deg, tilt_deg)
            self._scan_substep = 1
            self._scan_step_t  = time.time()
            self.get_logger().info(
                f'[{self._grid_idx+1:02d}/{len(self._grid)}] '
                f'pan={pan_deg:+.0f}°  tilt={tilt_deg:+.0f}°')

        elif self._scan_substep == 1:
            if time.time() - self._scan_step_t >= self._settle:
                self._scan_substep = 2

        elif self._scan_substep == 2:
            self._waiting_result = True
            self._match_found    = False
            self._capture_sent_t = time.time()
            self._pub_capture.publish(Bool(data=True))
            self._scan_substep = 3

        elif self._scan_substep == 3:
            if self._match_found:
                self.get_logger().info(
                    f'TARGET FOUND at [{self._grid_idx+1}/{len(self._grid)}] '
                    f'pan={pan_deg:+.0f}° tilt={tilt_deg:+.0f}°. → FINE_TUNE')
                self._state        = FINE_TUNE
                self._ft_start     = time.time()
                self._scan_substep = 0
            else:
                self._grid_idx    += 1
                self._scan_substep = 0

    # ── FINE_TUNE ─────────────────────────────────────────────────────────────

    def _run_fine_tune(self):
        elapsed = time.time() - self._ft_start

        if not hasattr(self, '_ft_last_capture'):
            self._ft_last_capture = 0.0
        if time.time() - self._ft_last_capture >= 0.2:
            self._pub_capture.publish(Bool(data=True))
            self._ft_last_capture = time.time()

        h_ok = abs(self._h_err) <= self._tol
        v_ok = abs(self._v_err) <= self._tol

        if (h_ok and v_ok) or elapsed >= self._ft_timeout:
            reason = 'centred' if (h_ok and v_ok) else 'timeout'
            self.get_logger().info(
                f'Fine-tune complete ({reason}). '
                f'h_err={self._h_err:.1f}px v_err={self._v_err:.1f}px')
            if hasattr(self, '_ft_last_capture'):
                del self._ft_last_capture
            self._state      = FIRE
            self._fire_start = time.time()
            return

        # Proportional correction
        # h_err > 0 → face is RIGHT of centre → pan RIGHT → decrease pan
        # (pan positive = left, so right correction = subtract)
        # v_err > 0 → face is BELOW centre   → tilt DOWN  → decrease tilt
        new_pan  = self._cur_pan  - self._gain_h * self._h_err
        new_tilt = self._cur_tilt - self._gain_v * self._v_err

        # Clamp to safe range
        new_pan  = max(-170.0, min(170.0, new_pan))
        new_tilt = max(-80.0,  min(80.0,  new_tilt))
        self._move_turret(new_pan, new_tilt)

    # ── FIRE ──────────────────────────────────────────────────────────────────

    def _run_fire(self):
        if time.time() - self._fire_start < self._laser_time:
            self._pub_laser.publish(Bool(data=True))
        else:
            self._pub_laser.publish(Bool(data=False))
            self.get_logger().info('Laser OFF. Task COMPLETE.')
            self._pub_complete.publish(Bool(data=True))
            self._state = DONE

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _move_turret(self, pan_deg: float, tilt_deg: float):
        """
        Publish turret position in robot-frame degrees.
        pan_deg : 0=forward, +=left, -=right
        tilt_deg: 0=level,   +=up,   -=down
        turret_gazebo_bridge converts these directly to radians (no offset).
        """
        self._cur_pan  = pan_deg
        self._cur_tilt = tilt_deg
        self._pub_pan.publish(Float32(data=float(pan_deg)))
        self._pub_tilt.publish(Float32(data=float(tilt_deg)))

    def _reset_scan(self):
        self._grid_idx       = 0
        self._waiting_result = False
        self._match_found    = False
        self._h_err          = 0.0
        self._v_err          = 0.0
        self._ft_start       = None
        self._fire_start     = None
        self._cur_pan        = 0.0
        self._cur_tilt       = 0.0
        for attr in ('_scan_substep', '_ft_last_capture', '_capture_sent_t'):
            if hasattr(self, attr):
                delattr(self, attr)


def main(args=None):
    rclpy.init(args=args)
    node = FaceTaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
