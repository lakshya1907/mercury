#!/usr/bin/env python3
"""
System Monitor Node
===================
Tracks which ROS2 nodes are running vs expected, monitors initialization
status, and publishes a system health summary on /system_status.

Topics published:
  /system_status  (std_msgs/String)  — JSON-encoded health report

Parameters:
  monitor_interval  (float, default 2.0)  — seconds between health checks
  expected_nodes    (list[str])           — nodes that must be present
"""

import json
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Nodes the workspace is expected to run (sim mode).
# Covers nodes from description, localization, planning, perception, hardware.
# ---------------------------------------------------------------------------
DEFAULT_EXPECTED_NODES = [
    '/behavior_server',
    '/bt_navigator',
    '/control_listener_node',
    '/controller_manager',
    '/controller_server',
    '/diff_drive_controller',
    '/ekf_filter_node',
    '/global_costmap/global_costmap',
    '/goal_decomposer',
    '/gz_ros_control',
    '/joint_state_broadcaster',
    '/lane_assist_node',
    '/lane_costmap',
    '/lifecycle_manager_localization',
    '/lifecycle_manager_navigation',
    '/local_costmap/local_costmap',
    '/monitoring_dashboard',
    '/planner_server',
    '/pothole_costmap',
    '/robot_state_publisher',
    '/ros_gz_bridge',
    '/rviz2',
    '/slam_toolbox',
    '/system_monitor_node',
    '/turret_controller',
    '/twist_to_stamped',
    '/watchdog_node',
    '/waypoint_detector_node',
]


class SystemMonitorNode(Node):
    """Continuously monitors the set of active ROS2 nodes."""

    def __init__(self) -> None:
        super().__init__('system_monitor_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('monitor_interval', 2.0)
        self.declare_parameter('expected_nodes', DEFAULT_EXPECTED_NODES)

        self._interval: float = (
            self.get_parameter('monitor_interval').get_parameter_value().double_value
        )
        self._expected: list[str] = (
            self.get_parameter('expected_nodes')
            .get_parameter_value()
            .string_array_value
        )

        # ── Publisher ────────────────────────────────────────────────────────
        self._pub = self.create_publisher(String, '/system_status', 10)

        # ── Launch-order tracking ────────────────────────────────────────────
        self._launch_order: list[dict] = []
        self._seen_nodes: set[str] = set()
        self._start_time = time.time()

        # ── Timer ────────────────────────────────────────────────────────────
        self.create_timer(self._interval, self._monitor_callback)

        self.get_logger().info(
            f'[SystemMonitor] Started — checking every {self._interval}s'
        )
        self.get_logger().info(
            f'[SystemMonitor] Expecting {len(self._expected)} nodes'
        )

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _get_running_nodes(self) -> list[str]:
        """Return list of node names currently visible to the ROS2 graph."""
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True, text=True, timeout=5.0
            )
            nodes = [
                line.strip()
                for line in result.stdout.splitlines()
                if line.strip()
            ]
            return nodes
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            self.get_logger().warn(f'[SystemMonitor] ros2 node list failed: {exc}')
            return []

    def _track_launch_order(self, running: list[str]) -> None:
        """Record first-seen timestamps for each node."""
        now = time.time()
        for node in running:
            if node not in self._seen_nodes:
                self._seen_nodes.add(node)
                elapsed = now - self._start_time
                self._launch_order.append(
                    {'node': node, 'detected_at_s': round(elapsed, 2)}
                )
                self.get_logger().info(
                    f'[SystemMonitor] NEW NODE detected: {node} '
                    f'(+{elapsed:.1f}s after monitor start)'
                )

    # ── Timer callback ────────────────────────────────────────────────────────

    def _monitor_callback(self) -> None:
        running = self._get_running_nodes()
        self._track_launch_order(running)

        running_set = set(running)
        expected_set = set(self._expected)

        missing = sorted(expected_set - running_set)
        unexpected = sorted(running_set - expected_set)
        healthy = sorted(expected_set & running_set)

        # ── Log missing nodes ────────────────────────────────────────────────
        if missing:
            for node in missing:
                self.get_logger().warn(
                    f'[SystemMonitor] MISSING node: {node}'
                )
        else:
            self.get_logger().debug('[SystemMonitor] All expected nodes running ✓')

        # ── Build status payload ─────────────────────────────────────────────
        status = {
            'timestamp': time.time(),
            'healthy': healthy,
            'missing': missing,
            'unexpected': unexpected,
            'total_running': len(running),
            'total_expected': len(self._expected),
            'all_ok': len(missing) == 0,
            'launch_order': self._launch_order,
        }

        msg = String()
        msg.data = json.dumps(status)
        self._pub.publish(msg)

        # ── Terminal summary ──────────────────────────────────────────────────
        self.get_logger().info(
            f'[SystemMonitor] Status → running={len(running)} '
            f'expected={len(self._expected)} '
            f'missing={len(missing)} '
            f'all_ok={status["all_ok"]}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemMonitorNode()
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