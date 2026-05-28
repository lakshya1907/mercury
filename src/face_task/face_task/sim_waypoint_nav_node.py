#!/usr/bin/env python3
"""
sim_waypoint_nav_node.py
========================
Simulation-only navigation node that:

  1. Sends the robot to WP-1 first (optional warmup waypoint on the track).
  2. Then sends it to WP-2  (-7.5, -41.0) — the face-detection zone.
  3. Once the robot arrives within `arrival_radius` of WP-2, it:
       a. Publishes a zero-velocity cmd_vel to stop the robot.
       b. Publishes a /waypoint_reached JSON event (same format as
          waypoint_detector_node) so that face_task_trigger_node fires.
  4. Waits for /face_task/done  (published by face_task_trigger_node
     when face detection is finished).
  5. Optionally sends the robot on to WP-3 once the face task is done.

The node uses Nav2's NavigateToPose action client to command the robot.
It does NOT replace waypoint_detector_node — it works alongside it.
Both nodes publish /waypoint_reached; face_task_trigger_node consumes
the first matching event regardless of source.

Parameters
----------
  wp1_x, wp1_y          float  WP-1 coordinates (default: 2.0, 0.0)
  wp2_x, wp2_y          float  WP-2 coordinates (default: -7.5, -41.0)
  wp3_x, wp3_y          float  WP-3 coordinates (default: 2.0, -55.0)
  arrival_radius        float  metres to count as "arrived" (default: 1.5)
  cmd_vel_topic         str    (default: /cmd_vel)
  navigate_to_wp3       bool   go to WP-3 after face task (default: True)
  nav_startup_delay_sec float  seconds to wait before Nav2 is ready (default: 15.0)
  skip_wp1              bool   skip WP-1 and go straight to WP-2 (default: False)
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

try:
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped
    _HAVE_NAV2 = True
except ImportError:
    _HAVE_NAV2 = False

try:
    from nav_msgs.msg import Odometry
    _HAVE_NAV = True
except ImportError:
    _HAVE_NAV = False


# ── State machine ──────────────────────────────────────────────────────────────
WAITING_FOR_NAV2  = 'WAITING_FOR_NAV2'
NAVIGATING_TO_WP1 = 'NAVIGATING_TO_WP1'
NAVIGATING_TO_WP2 = 'NAVIGATING_TO_WP2'
AT_WP2_FACE_TASK  = 'AT_WP2_FACE_TASK'
NAVIGATING_TO_WP3 = 'NAVIGATING_TO_WP3'
DONE              = 'DONE'


class SimWaypointNavNode(Node):

    def __init__(self) -> None:
        super().__init__('sim_waypoint_nav_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('wp1_x',                2.0)
        self.declare_parameter('wp1_y',                0.0)
        self.declare_parameter('wp2_x',               -7.5)
        self.declare_parameter('wp2_y',              -41.0)
        self.declare_parameter('wp3_x',                2.0)
        self.declare_parameter('wp3_y',              -55.0)
        self.declare_parameter('arrival_radius',       1.5)
        self.declare_parameter('cmd_vel_topic',       '/cmd_vel')
        self.declare_parameter('navigate_to_wp3',     True)
        self.declare_parameter('nav_startup_delay_sec', 15.0)
        self.declare_parameter('skip_wp1',            False)

        p = self.get_parameter
        self._wp1 = (p('wp1_x').value, p('wp1_y').value)
        self._wp2 = (p('wp2_x').value, p('wp2_y').value)
        self._wp3 = (p('wp3_x').value, p('wp3_y').value)
        self._radius          = p('arrival_radius').value
        self._nav_to_wp3      = p('navigate_to_wp3').value
        self._startup_delay   = p('nav_startup_delay_sec').value
        self._skip_wp1        = p('skip_wp1').value
        cmd_vel_topic         = p('cmd_vel_topic').value

        # ── State ─────────────────────────────────────────────────────────────
        self._state           = WAITING_FOR_NAV2
        self._start_time      = time.time()
        self._robot_x         = 0.0
        self._robot_y         = 0.0
        self._face_task_done  = False
        self._nav_goal_handle = None
        self._wp2_event_sent  = False

        # ── Nav2 action client ────────────────────────────────────────────────
        self._nav_client = None
        if _HAVE_NAV2:
            self._nav_client = ActionClient(
                self, NavigateToPose, 'navigate_to_pose')

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_wp_reached = self.create_publisher(
            String, '/waypoint_reached', 10)
        self._pub_cmd_vel = self.create_publisher(
            Twist, cmd_vel_topic, 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        if _HAVE_NAV:
            self.create_subscription(
                Odometry, '/diff_drive_controller/odom',
                self._odom_cb, 10)
            self.create_subscription(
                Odometry, '/odometry/filtered',
                self._odom_cb, 10)

        self.create_subscription(
            Bool, '/face_task/done', self._face_done_cb, 10)

        # ── Main loop timer (5 Hz) ────────────────────────────────────────────
        self.create_timer(0.2, self._loop)

        self.get_logger().info(
            f'[SimWaypointNav] Started. '
            f'WP-1={self._wp1}, WP-2={self._wp2}, WP-3={self._wp3}, '
            f'radius={self._radius}m. '
            f'Waiting {self._startup_delay}s for Nav2...')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: 'Odometry') -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def _face_done_cb(self, msg: Bool) -> None:
        if msg.data and self._state == AT_WP2_FACE_TASK:
            self.get_logger().info(
                '[SimWaypointNav] /face_task/done received. '
                'Face detection complete — robot stays at WP-2.')
            self._face_task_done = True

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _loop(self) -> None:
        if self._state == WAITING_FOR_NAV2:
            elapsed = time.time() - self._start_time
            if elapsed >= self._startup_delay:
                self.get_logger().info(
                    '[SimWaypointNav] Nav2 startup delay passed. '
                    'Beginning navigation sequence.')
                if self._skip_wp1:
                    self._navigate_to(self._wp2, 'WP-2')
                    self._state = NAVIGATING_TO_WP2
                else:
                    self._navigate_to(self._wp1, 'WP-1')
                    self._state = NAVIGATING_TO_WP1

        elif self._state == NAVIGATING_TO_WP1:
            dist = self._dist_to(self._wp1)
            if dist <= self._radius:
                self.get_logger().info(
                    f'[SimWaypointNav] Arrived at WP-1 (dist={dist:.2f}m). '
                    f'Navigating to WP-2...')
                self._navigate_to(self._wp2, 'WP-2')
                self._state = NAVIGATING_TO_WP2

        elif self._state == NAVIGATING_TO_WP2:
            dist = self._dist_to(self._wp2)
            if dist <= self._radius:
                self._on_wp2_arrived()

        elif self._state == AT_WP2_FACE_TASK:
            # Keep robot stopped while face task runs
            self._stop_robot()

            if self._face_task_done:
                self.get_logger().info(
                    '[SimWaypointNav] Face task complete. '
                    'Robot remains stationary at WP-2 as requested.')
                self._state = DONE

        elif self._state == NAVIGATING_TO_WP3:
            dist = self._dist_to(self._wp3)
            if dist <= self._radius:
                self.get_logger().info(
                    f'[SimWaypointNav] Arrived at WP-3. Mission complete.')
                self._state = DONE

        elif self._state == DONE:
            pass  # mission finished

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _on_wp2_arrived(self) -> None:
        self.get_logger().info(
            f'[SimWaypointNav] *** WP-2 REACHED at '
            f'({self._robot_x:.2f}, {self._robot_y:.2f}) ***')

        # Stop the robot
        self._stop_robot()

        # Publish waypoint_reached event (same JSON format as waypoint_detector_node)
        if not self._wp2_event_sent:
            self._wp2_event_sent = True
            event = {
                'event': 'waypoint_reached',
                'waypoint': {
                    'index': 2,
                    'name': 'WP-2',
                    'x': self._wp2[0],
                    'y': self._wp2[1],
                    'radius': self._radius,
                    'reached': True,
                    'reach_count': 1,
                    'reached_at': time.time(),
                },
                'robot_x': self._robot_x,
                'robot_y': self._robot_y,
                'distance': self._dist_to(self._wp2),
                'timestamp': time.time(),
            }
            msg = String()
            msg.data = json.dumps(event)
            self._pub_wp_reached.publish(msg)
            self.get_logger().info(
                '[SimWaypointNav] Published /waypoint_reached for WP-2. '
                'face_task_trigger_node will start face detection.')

        self._state = AT_WP2_FACE_TASK

    def _navigate_to(self, wp: tuple, name: str) -> None:
        """Send a NavigateToPose goal via Nav2."""
        if not _HAVE_NAV2 or self._nav_client is None:
            self.get_logger().warn(
                f'[SimWaypointNav] nav2_msgs not available — '
                f'skipping Nav2 goal for {name}. '
                f'Move robot manually or use teleop.')
            return

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                f'[SimWaypointNav] NavigateToPose server not available '
                f'for {name}. Will detect arrival by odometry proximity.')
            return

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wp[0])
        pose.pose.position.y = float(wp[1])
        pose.pose.position.z = 0.0
        # Face forward (identity quaternion)
        pose.pose.orientation.w = 1.0
        goal.pose = pose

        self.get_logger().info(
            f'[SimWaypointNav] Sending Nav2 goal → {name} '
            f'({wp[0]:.2f}, {wp[1]:.2f})')

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self.get_logger().info(
                f'[SimWaypointNav] Nav2 goal accepted for {name}'))

    def _stop_robot(self) -> None:
        self._pub_cmd_vel.publish(Twist())

    def _dist_to(self, wp: tuple) -> float:
        return math.sqrt(
            (self._robot_x - wp[0]) ** 2 +
            (self._robot_y - wp[1]) ** 2)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimWaypointNavNode()
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
