#!/usr/bin/env python3
"""
face_task_trigger_node.py
=========================
Bridge node: connects waypoint_detector_node → face_task pipeline.

Subscribes
----------
  /waypoint_reached      (std_msgs/String)  JSON from watchdog_monitor
  /face_task/complete    (std_msgs/Bool)    result from face_task_node

Publishes
---------
  /face_task/start       (std_msgs/Bool)    True  → wake face_task_node
  /face_task/done        (std_msgs/Bool)    True  → navigation may go to WP-3
  /face_task/state       (std_msgs/String)  human-readable state for monitoring
  /cmd_vel               (geometry_msgs/Twist) zero-twist to hold vehicle still

Parameters
----------
  trigger_waypoint_name   str    default: "WP-2"
  trigger_waypoint_index  int    default: 2  (1-based, fallback if name missing)
  hard_timeout_sec        float  default: 55.0  (5 s buffer before 60 s penalty)
  stop_vehicle_on_trigger bool   default: true
  cmd_vel_topic           str    default: /cmd_vel
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from geometry_msgs.msg import Twist
    _HAVE_TWIST = True
except ImportError:
    _HAVE_TWIST = False
    Twist = None  # type: ignore[assignment,misc]

STATE_IDLE      = 'IDLE'
STATE_TRIGGERED = 'TRIGGERED'
STATE_COMPLETE  = 'COMPLETE'


class FaceTaskTriggerNode(Node):

    def __init__(self) -> None:
        super().__init__('face_task_trigger_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('trigger_waypoint_name',   'WP-2')
        self.declare_parameter('trigger_waypoint_index',  2)
        self.declare_parameter('hard_timeout_sec',        55.0)
        self.declare_parameter('stop_vehicle_on_trigger', True)
        self.declare_parameter('cmd_vel_topic',           '/cmd_vel')

        self._trigger_name = self.get_parameter(
            'trigger_waypoint_name').get_parameter_value().string_value
        self._trigger_idx  = self.get_parameter(
            'trigger_waypoint_index').get_parameter_value().integer_value
        self._hard_timeout = self.get_parameter(
            'hard_timeout_sec').get_parameter_value().double_value
        self._stop_vehicle = self.get_parameter(
            'stop_vehicle_on_trigger').get_parameter_value().bool_value
        cmd_vel_topic      = self.get_parameter(
            'cmd_vel_topic').get_parameter_value().string_value

        # ── Internal state ────────────────────────────────────────────────────
        self._state        = STATE_IDLE
        self._task_start_t = None
        self._task_result  = None

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_start = self.create_publisher(Bool,   '/face_task/start', 10)
        self._pub_done  = self.create_publisher(Bool,   '/face_task/done',  10)
        self._pub_state = self.create_publisher(String, '/face_task/state', 10)

        if self._stop_vehicle and _HAVE_TWIST:
            self._pub_cmdvel = self.create_publisher(Twist, cmd_vel_topic, 10)
        else:
            self._pub_cmdvel = None

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            String, '/waypoint_reached', self._waypoint_cb, 10)
        self.create_subscription(
            Bool, '/face_task/complete', self._complete_cb, 10)

        # ── 10 Hz watchdog timer ──────────────────────────────────────────────
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            f'[FaceTaskTrigger] Ready — trigger="{self._trigger_name}" '
            f'(idx={self._trigger_idx}), timeout={self._hard_timeout}s')
        self._publish_state(STATE_IDLE)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _waypoint_cb(self, msg: String) -> None:
        if self._state != STATE_IDLE:
            return

        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'[FaceTaskTrigger] Bad JSON: {exc}')
            return

        wp        = event.get('waypoint', {})
        wp_name   = wp.get('name', '')
        wp_index  = wp.get('index', -1)

        if wp_name != self._trigger_name and wp_index != self._trigger_idx:
            return

        self.get_logger().info(
            f'[FaceTaskTrigger] WP-2 reached ("{wp_name}", idx={wp_index}). '
            f'Stopping vehicle and starting face task.')

        self._state        = STATE_TRIGGERED
        self._task_start_t = time.time()
        self._task_result  = None

        self._stop_robot()

        start = Bool()
        start.data = True
        self._pub_start.publish(start)
        self._publish_state(STATE_TRIGGERED)

    def _complete_cb(self, msg: Bool) -> None:
        if self._state != STATE_TRIGGERED:
            return

        self._task_result = msg.data
        elapsed = time.time() - self._task_start_t
        result_str = 'SUCCESS' if msg.data else 'NOT FOUND'
        self.get_logger().info(
            f'[FaceTaskTrigger] Face task complete: {result_str} in {elapsed:.1f}s')
        self._finish_task()

    # ── Watchdog ──────────────────────────────────────────────────────────────

    def _watchdog(self) -> None:
        if self._state != STATE_TRIGGERED:
            return

        self._stop_robot()

        elapsed = time.time() - self._task_start_t

        # Log once every 5 seconds (not every tick that's a multiple of 5)
        if not hasattr(self, '_last_log_t'):
            self._last_log_t = 0.0
        if elapsed - self._last_log_t >= 5.0:
            self._last_log_t = elapsed
            remaining = self._hard_timeout - elapsed
            self.get_logger().info(
                f'[FaceTaskTrigger] Scanning... {elapsed:.0f}s elapsed, '
                f'{remaining:.0f}s remaining')

        if elapsed >= self._hard_timeout:
            self.get_logger().warn(
                f'[FaceTaskTrigger] Hard timeout after {elapsed:.1f}s — '
                f'forcing exit to WP-3.')
            self._task_result = False
            self._finish_task()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _finish_task(self) -> None:
        self._state = STATE_COMPLETE
        done = Bool()
        done.data = True
        self._pub_done.publish(done)
        self._publish_state(STATE_COMPLETE)
        self.get_logger().info(
            '[FaceTaskTrigger] /face_task/done=True published. '
            'Navigation may proceed to WP-3.')

    def _stop_robot(self) -> None:
        if self._pub_cmdvel is not None and _HAVE_TWIST:
            self._pub_cmdvel.publish(Twist())

    def _publish_state(self, state: str) -> None:
        self._state = state
        payload = {
            'node':              'face_task_trigger',
            'state':             state,
            'trigger_waypoint':  self._trigger_name,
            'task_result':       self._task_result,
            'elapsed':           round(time.time() - self._task_start_t, 2)
                                 if self._task_start_t else None,
            'timestamp':         time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._pub_state.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaceTaskTriggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
