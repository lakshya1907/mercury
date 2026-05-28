#!/usr/bin/env python3
"""
face_task_node.py
=================
Brain node for Plan 3 face recognition task.

State machine:
    IDLE        -- waiting for /face_task/start
    SCANNING    -- iterating through 18 (H×V) turret positions, capturing + recognising
    FINE_TUNE   -- proportional controller to centre face in frame
    FIRE        -- activate laser, wait, publish done
    DONE        -- publish /face_task/complete, return to IDLE

Subscribes:
    /face_task/start           (std_msgs/Bool)   -- True starts the task
    /face/match_found          (std_msgs/Bool)
    /face/horizontal_error     (std_msgs/Float32)
    /face/vertical_error       (std_msgs/Float32)

Publishes:
    /face/capture_request      (std_msgs/Bool)   -- trigger vision node
    /turret/pan_deg            (std_msgs/Float32) -- horizontal servo target (degrees)
    /turret/tilt_deg           (std_msgs/Float32) -- vertical servo target (degrees)
    /laser/fire                (std_msgs/Bool)
    /face_task/complete        (std_msgs/Bool)   -- True when task finished

Parameters (all tunable at launch):
    h_positions_deg   list of 6 horizontal angles  default: [-75,-45,-15,15,45,75]
    v_positions_deg   list of 3 vertical angles    default: [40,25,59]  (mid,low,high)
    settle_time_sec   turret settle delay           default: 0.5
    fine_tune_px_tol  pixel tolerance for fine-tune default: 20.0
    fine_tune_gain_h  proportional gain horizontal  default: 0.05 (deg/px)
    fine_tune_gain_v  proportional gain vertical    default: 0.05 (deg/px)
    fine_tune_timeout timeout for fine-tune phase   default: 5.0
    laser_on_time_sec laser fire duration           default: 3.0
    pan_centre_deg    servo centre for pan          default: 90.0
    tilt_centre_deg   servo centre for tilt         default: 90.0
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
        self.declare_parameter('h_positions_deg',   [-135.0, -90.0, -45.0, 0.0, 45.0,  90.0, 135.0])
        self.declare_parameter('v_positions_deg',   [59.0, 40.0, 25.0])   # high, mid, low
        self.declare_parameter('settle_time_sec',   0.5)
        self.declare_parameter('fine_tune_px_tol',  20.0)
        self.declare_parameter('fine_tune_gain_h',  0.05)
        self.declare_parameter('fine_tune_gain_v',  0.05)
        self.declare_parameter('fine_tune_timeout', 5.0)
        self.declare_parameter('laser_on_time_sec', 3.0)
        self.declare_parameter('pan_centre_deg',    90.0)
        self.declare_parameter('tilt_centre_deg',   90.0)

        self._h_pos      = self.get_parameter('h_positions_deg').value
        self._v_pos      = self.get_parameter('v_positions_deg').value
        self._settle     = self.get_parameter('settle_time_sec').value
        self._tol        = self.get_parameter('fine_tune_px_tol').value
        self._gain_h     = self.get_parameter('fine_tune_gain_h').value
        self._gain_v     = self.get_parameter('fine_tune_gain_v').value
        self._ft_timeout = self.get_parameter('fine_tune_timeout').value
        self._laser_time = self.get_parameter('laser_on_time_sec').value
        self._pan_ctr    = self.get_parameter('pan_centre_deg').value
        self._tilt_ctr   = self.get_parameter('tilt_centre_deg').value

        # ── Build 18-position scan grid ────────────────────────────────────────
        # Order: V[0](mid) full H sweep → V[1](low) full H sweep → V[2](high) full H sweep
        self._grid = []

        for row_idx, v in enumerate(self._v_pos):

            if row_idx % 2 == 0:
                h_scan = self._h_pos
            else:
                h_scan = list(reversed(self._h_pos))

            for h in h_scan:
                self._grid.append((h, v))
        # ── State ─────────────────────────────────────────────────────────────
        self._state          = IDLE
        self._grid_idx       = 0
        self._waiting_result = False
        self._match_found    = False
        self._h_err          = 0.0
        self._v_err          = 0.0
        self._ft_start       = None
        self._fire_start     = None

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Bool,    '/face_task/start',        self._start_cb, 10)
        self.create_subscription(Bool,    '/face/match_found',       self._match_cb, 10)
        self.create_subscription(Float32, '/face/horizontal_error',  self._herr_cb,  10)
        self.create_subscription(Float32, '/face/vertical_error',    self._verr_cb,  10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_capture  = self.create_publisher(Bool,    '/face/capture_request', 10)
        self._pub_pan      = self.create_publisher(Float32, '/turret/pan_deg',       10)
        self._pub_tilt     = self.create_publisher(Float32, '/turret/tilt_deg',      10)
        self._pub_laser    = self.create_publisher(Bool,    '/laser/fire',           10)
        self._pub_complete = self.create_publisher(Bool,    '/face_task/complete',   10)

        # ── Main control loop at 10 Hz ─────────────────────────────────────────
        self._loop_timer = self.create_timer(0.1, self._loop)

        self.get_logger().info('FaceTaskNode ready. Waiting for /face_task/start ...')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _start_cb(self, msg: Bool):
        if msg.data and self._state == IDLE:
            self.get_logger().info('Task START received. Beginning 21-image grid scan.')
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

        elif self._state == DONE:
            pass  # stay here until reset

    # ── SCANNING phase ────────────────────────────────────────────────────────
    def _run_scanning(self):
        """
        Step through the 21-position grid one position at a time.
        For each position:
          1. Command turret to (H, V)
          2. Wait settle_time
          3. Trigger capture
          4. Wait for match result
          5. If match → transition to FINE_TUNE
          6. Else → advance to next position
        """
        # If we're waiting for vision node to respond, do nothing —
        # BUT apply a per-position timeout so a silent recognition node
        # (e.g. no target embedding loaded) cannot stall the entire scan.
        if self._waiting_result:
            capture_timeout = self._settle + 0.7   # settle + 2 s grace
            if hasattr(self, '_capture_sent_t') and \
                    time.time() - self._capture_sent_t > capture_timeout:
                self.get_logger().warn(
                    f'[{self._grid_idx+1:02d}/21] No response from face_recognition_node '
                    f'after {capture_timeout:.1f}s — treating as no-match and advancing.')
                self._waiting_result = False
                self._match_found    = False
            else:
                return

        # All positions exhausted without finding target
        if self._grid_idx >= len(self._grid):
            self.get_logger().warn(
                'Scan complete — target NOT found in all 21 positions. '
                'Publishing task complete (no match).')
            self._pub_complete.publish(Bool(data=False))
            self._state = DONE
            return

        h_deg, v_deg = self._grid[self._grid_idx]

        # ── Substep tracking via a small internal sub-state ────────────────
        # We use _scan_substep: 0=move, 1=settling, 2=capture, 3=wait_result
        if not hasattr(self, '_scan_substep'):
            self._scan_substep  = 0
            self._scan_step_t   = None

        if self._scan_substep == 0:
            # Command turret
            self._move_turret(h_deg, v_deg)
            self._scan_substep = 1
            self._scan_step_t  = time.time()
            self.get_logger().info(
                f'[{self._grid_idx+1:02d}/21] Moving turret → H={h_deg}° V={v_deg}°')

        elif self._scan_substep == 1:
            # Wait for settle
            if time.time() - self._scan_step_t >= self._settle:
                self._scan_substep = 2

        elif self._scan_substep == 2:
            # Trigger capture + recognition
            self._waiting_result  = True
            self._match_found     = False
            self._capture_sent_t  = time.time()   # for per-position timeout
            self._pub_capture.publish(Bool(data=True))
            self._scan_substep = 3

        elif self._scan_substep == 3:
            # Result arrived (_waiting_result cleared by _match_cb)
            if self._match_found:
                self.get_logger().info(
                    f'TARGET FOUND at grid position [{self._grid_idx+1}/21] '
                    f'H={h_deg}° V={v_deg}°. Transitioning to FINE_TUNE.')
                self._state        = FINE_TUNE
                self._ft_start     = time.time()
                self._scan_substep = 0
            else:
                # Advance to next position
                self._grid_idx    += 1
                self._scan_substep = 0

    # ── FINE_TUNE phase ───────────────────────────────────────────────────────
    def _run_fine_tune(self):
        """
        Proportional controller: adjust turret until face is centred
        within ±_tol pixels on both axes.
        Timeout after _ft_timeout seconds → fire anyway.
        """
        elapsed = time.time() - self._ft_start

        # Trigger a fresh capture every 0.2s to get updated pixel errors
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
            # Clean up
            if hasattr(self, '_ft_last_capture'):
                del self._ft_last_capture
            self._state      = FIRE
            self._fire_start = time.time()
            return

        # Proportional correction
        # h_err > 0 → face is right → increase pan (positive direction)
        # v_err > 0 → face is below → increase tilt
        current_h, current_v = self._grid[self._grid_idx]
        new_h = current_h + self._gain_h * self._h_err
        new_v = current_v + self._gain_v * self._v_err

        # Clamp to safe range
        new_h = max(-135.0, min(135.0, new_h))
        new_v = max(0.0,   min(120.0, new_v))

        self._move_turret(new_h, new_v)

    # ── FIRE phase ────────────────────────────────────────────────────────────
    def _run_fire(self):
        """Fire laser for laser_on_time_sec, then publish complete."""
        if time.time() - self._fire_start < self._laser_time:
            self._pub_laser.publish(Bool(data=True))
        else:
            self._pub_laser.publish(Bool(data=False))
            self.get_logger().info('Laser OFF. Task COMPLETE.')
            self._pub_complete.publish(Bool(data=True))
            self._state = DONE

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _move_turret(self, pan_deg: float, tilt_deg: float):
        """Publish turret position commands."""
        # Convert from scan angles to servo angles
        # pan_centre_deg is the servo neutral (facing forward)
        servo_pan  = self._pan_ctr  + pan_deg   # add offset from centre
        servo_tilt = self._tilt_ctr + tilt_deg  # add offset from centre
        self._pub_pan.publish(Float32(data=float(servo_pan)))
        self._pub_tilt.publish(Float32(data=float(servo_tilt)))

    def _reset_scan(self):
        """Reset state for a fresh scan."""
        self._grid_idx       = 0
        self._waiting_result = False
        self._match_found    = False
        self._h_err          = 0.0
        self._v_err          = 0.0
        self._ft_start       = None
        self._fire_start     = None
        if hasattr(self, '_scan_substep'):
            del self._scan_substep
        if hasattr(self, '_ft_last_capture'):
            del self._ft_last_capture


def main(args=None):
    rclpy.init(args=args)
    node = FaceTaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
