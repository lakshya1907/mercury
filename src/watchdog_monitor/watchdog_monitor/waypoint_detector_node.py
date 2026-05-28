#!/usr/bin/env python3
"""
Waypoint Detector Node
======================
Subscribes to odometry, computes Euclidean distance to each predefined
waypoint, and publishes an event the moment the robot enters the arrival
radius of each waypoint.

Each waypoint is only fired ONCE per run (it is "consumed" on detection).
The node re-arms a waypoint if it drifts >2× the radius back out, so the
same waypoint can be re-detected if the robot does a second pass.

Topics subscribed:
  /diff_drive_controller/odom  (nav_msgs/Odometry)  — primary pose source
  /odometry/filtered           (nav_msgs/Odometry)  — EKF-fused fallback

Topics published:
  /waypoint_reached   (std_msgs/String)  — JSON event on each detection
  /waypoint_status    (std_msgs/String)  — periodic JSON overview of all WPs

Parameters:
  waypoints           (list[float])  — flat list: [x1,y1, x2,y2, x3,y3, ...]
  waypoint_names      (list[str])    — human-readable label per waypoint
  arrival_radius      (float)        — metres — distance to count as "reached"
  status_interval     (float)        — seconds between /waypoint_status publishes
  odom_topic          (str)          — which odometry topic to subscribe to
  spawn_x             (float)        — world X of the robot spawn point (metres)
  spawn_y             (float)        — world Y of the robot spawn point (metres)
"""

import json
import math
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from nav_msgs.msg import Odometry
    _HAVE_NAV = True
except ImportError:
    _HAVE_NAV = False
    Odometry = None  # type: ignore[assignment,misc]


# ---------------------------------------------------------------------------
# Default waypoints (in the sim/map frame).
# These match the UGVC track world roughly — adjust to your actual map.
# Format: [x, y]  (z is ignored — 2-D comparison)
# ---------------------------------------------------------------------------
DEFAULT_WAYPOINTS_FLAT = [
    2.0,  0.0,   # WP-1
    2.0,  4.0,   # WP-2
    0.0,  4.0,   # WP-3
]
DEFAULT_WAYPOINT_NAMES = ['WP-1', 'WP-2', 'WP-3']
DEFAULT_ARRIVAL_RADIUS = 0.5    # metres
DEFAULT_STATUS_INTERVAL = 1.0   # seconds


class Waypoint:
    """Internal state for a single waypoint."""

    def __init__(self, idx: int, name: str, x: float, y: float, radius: float) -> None:
        self.idx = idx
        self.name = name
        self.x = x
        self.y = y
        self.radius = radius
        self.reached = False
        self.reached_at: float | None = None
        self.reach_count = 0   # how many times this WP has been reached

    def distance_to(self, rx: float, ry: float) -> float:
        return math.sqrt((rx - self.x) ** 2 + (ry - self.y) ** 2)

    def to_dict(self) -> dict[str, Any]:
        return {
            'index': self.idx,
            'name': self.name,
            'x': self.x,
            'y': self.y,
            'radius': self.radius,
            'reached': self.reached,
            'reach_count': self.reach_count,
            'reached_at': self.reached_at,
        }


class WaypointDetectorNode(Node):
    """Detects when the robot arrives at predefined waypoints."""

    def __init__(self) -> None:
        super().__init__('waypoint_detector_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('waypoints', DEFAULT_WAYPOINTS_FLAT)
        self.declare_parameter('waypoint_names', DEFAULT_WAYPOINT_NAMES)
        self.declare_parameter('arrival_radius', DEFAULT_ARRIVAL_RADIUS)
        self.declare_parameter('status_interval', DEFAULT_STATUS_INTERVAL)
        self.declare_parameter('odom_topic', '/diff_drive_controller/odom')
        self.declare_parameter('spawn_x', 0.0)   # world-frame X at spawn
        self.declare_parameter('spawn_y', 0.0)   # world-frame Y at spawn

        waypoints_flat: list[float] = (
            self.get_parameter('waypoints').get_parameter_value().double_array_value
        )
        names: list[str] = (
            self.get_parameter('waypoint_names')
            .get_parameter_value().string_array_value
        )
        radius: float = (
            self.get_parameter('arrival_radius').get_parameter_value().double_value
        )
        self._status_interval: float = (
            self.get_parameter('status_interval').get_parameter_value().double_value
        )
        odom_topic: str = (
            self.get_parameter('odom_topic').get_parameter_value().string_value
        )
        self._spawn_x: float = (
            self.get_parameter('spawn_x').get_parameter_value().double_value
        )
        self._spawn_y: float = (
            self.get_parameter('spawn_y').get_parameter_value().double_value
        )

        # ── Build waypoint objects ────────────────────────────────────────────
        if len(waypoints_flat) % 2 != 0:
            self.get_logger().error(
                '[WaypointDetector] waypoints param must have an even number of values '
                '(pairs of x,y). Ignoring last value.'
            )
            waypoints_flat = list(waypoints_flat[:-1])

        self._waypoints: list[Waypoint] = []
        for i, (x, y) in enumerate(zip(waypoints_flat[::2], waypoints_flat[1::2])):
            name = names[i] if i < len(names) else f'WP-{i + 1}'
            wp = Waypoint(idx=i + 1, name=name, x=x, y=y, radius=radius)
            self._waypoints.append(wp)

        self.get_logger().info(
            f'[WaypointDetector] Loaded {len(self._waypoints)} waypoints '
            f'with arrival_radius={radius}m'
        )
        self.get_logger().info(
            f'[WaypointDetector] Spawn offset: ({self._spawn_x:.2f}, {self._spawn_y:.2f}) — '
            f'waypoints are in world coordinates'
        )
        for wp in self._waypoints:
            self.get_logger().info(
                f'  {wp.name}: ({wp.x:.2f}, {wp.y:.2f})'
            )

        # ── Robot pose (initialised to spawn position, not 0,0) ───────────────
        self._robot_x: float = self._spawn_x
        self._robot_y: float = self._spawn_y
        self._pose_received = False

        # ── Publishers ────────────────────────────────────────────────────────
        self._event_pub = self.create_publisher(String, '/waypoint_reached', 10)
        self._status_pub = self.create_publisher(String, '/waypoint_status', 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        if _HAVE_NAV:
            self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
            # EKF-fused fallback — only used when primary odom is the same topic
            if odom_topic != '/odometry/filtered':
                self.create_subscription(
                    Odometry, '/odometry/filtered', self._odom_cb, 10
                )
        else:
            self.get_logger().warn(
                '[WaypointDetector] nav_msgs not available — '
                'pose will not be updated from odometry'
            )

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(0.1, self._detection_callback)   # 10 Hz detection loop
        self.create_timer(self._status_interval, self._status_callback)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: 'Odometry') -> None:
        # Odometry is relative to where the robot started (always 0,0 at boot).
        # Adding the spawn offset converts it to world/Gazebo coordinates so
        # that waypoints can be specified in world-frame directly.
        self._robot_x = self._spawn_x + msg.pose.pose.position.x
        self._robot_y = self._spawn_y + msg.pose.pose.position.y
        self._pose_received = True

    def _detection_callback(self) -> None:
        if not self._pose_received:
            return

        rx, ry = self._robot_x, self._robot_y

        for wp in self._waypoints:
            dist = wp.distance_to(rx, ry)

            # ── Arrival detection ─────────────────────────────────────────────
            if not wp.reached and dist <= wp.radius:
                wp.reached = True
                wp.reached_at = time.time()
                wp.reach_count += 1

                self.get_logger().info(
                    f'[WaypointDetector] *** WAYPOINT REACHED: {wp.name} '
                    f'(#{wp.reach_count}) at robot=({rx:.3f},{ry:.3f}) '
                    f'wp=({wp.x:.2f},{wp.y:.2f}) dist={dist:.3f}m ***'
                )

                event = {
                    'event': 'waypoint_reached',
                    'waypoint': wp.to_dict(),
                    'robot_x': rx,
                    'robot_y': ry,
                    'distance': round(dist, 4),
                    'timestamp': wp.reached_at,
                }
                msg = String()
                msg.data = json.dumps(event)
                self._event_pub.publish(msg)

            # ── Re-arm: allow re-detection if robot leaves the zone ───────────
            elif wp.reached and dist > wp.radius * 2.0:
                wp.reached = False
                self.get_logger().debug(
                    f'[WaypointDetector] {wp.name} re-armed '
                    f'(robot moved {dist:.2f}m away)'
                )

    def _status_callback(self) -> None:
        reached_count = sum(1 for wp in self._waypoints if wp.reach_count > 0)
        all_reached = reached_count == len(self._waypoints)

        status = {
            'timestamp': time.time(),
            'robot_x': round(self._robot_x, 4),
            'robot_y': round(self._robot_y, 4),
            'pose_received': self._pose_received,
            'waypoints': [wp.to_dict() for wp in self._waypoints],
            'total': len(self._waypoints),
            'reached_at_least_once': reached_count,
            'all_completed': all_reached,
        }

        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

        if all_reached:
            self.get_logger().info(
                '[WaypointDetector] ✓ All waypoints have been reached!'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointDetectorNode()
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