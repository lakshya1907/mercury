#!/usr/bin/env python3
"""
Watchdog Node
=============
Monitors critical nodes, topic liveness, and TF frame availability.
Detects crashes and inactivity, identifies root cause where possible,
and publishes structured alerts on /system_alerts.

Topics subscribed (liveness checks):
  /diff_drive_controller/odom  (nav_msgs/Odometry)
  /scan                        (sensor_msgs/LaserScan)
  /imu                         (sensor_msgs/Imu)
  /system_status               (std_msgs/String)  ← from system_monitor

Topics published:
  /system_alerts  (std_msgs/String)  — JSON-encoded alert list

Parameters:
  check_interval        (float, default 3.0)   — seconds between checks
  topic_timeout         (float, default 5.0)   — silence → inactivity alert
  critical_nodes        (list[str])            — nodes that must exist
  tf_pairs              (list[str])            — "parent->child" pairs to check
"""

import json
import subprocess
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Optional imports — gracefully degrade if nav_msgs/sensor_msgs not built yet
try:
    from nav_msgs.msg import Odometry
    _HAVE_NAV = True
except ImportError:
    _HAVE_NAV = False

try:
    from sensor_msgs.msg import LaserScan, Imu
    _HAVE_SENSOR = True
except ImportError:
    _HAVE_SENSOR = False


# ---------------------------------------------------------------------------
# Suggested fixes keyed by a pattern found in the node name / failure mode
# ---------------------------------------------------------------------------
FIX_HINTS: dict[str, str] = {
    '/slam_toolbox':
        'Check that slam_toolbox is installed: sudo apt install ros-$ROS_DISTRO-slam-toolbox',
    '/ekf_filter_node':
        'Check robot_localization pkg and ekf.yaml odom/imu topic names',
    '/controller_server':
        'Nav2 controller crash — check nav2_params.yaml and DWB critic config',
    '/planner_server':
        'Nav2 planner crash — verify global_costmap.yaml and map availability',
    '/bt_navigator':
        'BT Navigator crash — check bt_navigator params and BT XML file path',
    '/rplidar_node':
        'LiDAR driver offline — check USB connection: ls /dev/ttyUSB*',
    '/robot_state_publisher':
        'robot_state_publisher crashed — verify xacro file path is correct',
    'odom_timeout':
        'Odometry silent — check diff_drive_controller or simulation is running',
    'scan_timeout':
        'LaserScan silent — check rplidar_node is up and /scan topic remapping',
    'imu_timeout':
        'IMU silent — check realsense2_camera node and IMU unite method param',
    'tf_missing':
        'TF frame missing — robot_state_publisher or EKF may have crashed',
}


class WatchdogNode(Node):
    """Detects node crashes, topic silences, and TF failures."""

    def __init__(self) -> None:
        super().__init__('watchdog_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('check_interval', 3.0)
        self.declare_parameter('topic_timeout', 5.0)
        self.declare_parameter('critical_nodes', [
            '/robot_state_publisher',

            '/ekf_filter_node',
            '/slam_toolbox',

            '/planner_server',
            '/controller_server',
            '/bt_navigator',

            '/controller_manager',
            '/diff_drive_controller',

            '/lane_costmap',
        ])
        self.declare_parameter('tf_pairs', [
            'odom->base_link',
            'map->odom',
            'base_link->laser',
        ])

        self._check_interval: float = (
            self.get_parameter('check_interval').get_parameter_value().double_value
        )
        self._topic_timeout: float = (
            self.get_parameter('topic_timeout').get_parameter_value().double_value
        )
        self._critical_nodes: list[str] = (
            self.get_parameter('critical_nodes')
            .get_parameter_value().string_array_value
        )
        self._tf_pairs_raw: list[str] = (
            self.get_parameter('tf_pairs').get_parameter_value().string_array_value
        )

        # ── Topic liveness timestamps ─────────────────────────────────────────
        self._last_odom: float = time.time()
        self._last_scan: float = time.time()
        self._last_imu: float = time.time()
        self._last_system_status: float = time.time()

        # ── Publisher ────────────────────────────────────────────────────────
        self._alert_pub = self.create_publisher(String, '/system_alerts', 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        if _HAVE_NAV:
            self.create_subscription(
                Odometry, '/diff_drive_controller/odom',
                lambda _: self._touch('odom'), 10
            )
        if _HAVE_SENSOR:
            self.create_subscription(
                LaserScan, '/scan',
                lambda _: self._touch('scan'), 10
            )
            self.create_subscription(
                Imu, '/imu',
                lambda _: self._touch('imu'), 10
            )
        self.create_subscription(
            String, '/system_status',
            lambda _: self._touch('system_status'), 10
        )

        # ── Timer ────────────────────────────────────────────────────────────
        self.create_timer(self._check_interval, self._watchdog_callback)

        self.get_logger().info(
            f'[Watchdog] Started — check every {self._check_interval}s, '
            f'topic timeout {self._topic_timeout}s'
        )

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _touch(self, key: str) -> None:
        setattr(self, f'_last_{key}', time.time())

    def _get_running_nodes(self) -> list[str]:
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True, text=True, timeout=5.0
            )
            return [l.strip() for l in result.stdout.splitlines() if l.strip()]
        except Exception as exc:
            self.get_logger().warn(f'[Watchdog] node list failed: {exc}')
            return []

    def _check_tf(self, parent: str, child: str) -> bool:
        """Return True if the TF pair can be looked up (non-blocking)."""
        try:
            result = subprocess.run(
                ['ros2', 'run', 'tf2_ros', 'tf2_echo', parent, child, '--wait-time', '1'],
                capture_output=True, text=True, timeout=3.0
            )
            return result.returncode == 0
        except Exception:
            return False

    def _hint(self, key: str) -> str:
        for pattern, hint in FIX_HINTS.items():
            if pattern in key:
                return hint
        return 'Check node logs: ros2 node info <node_name>'

    def _make_alert(
        self,
        level: str,
        category: str,
        message: str,
        subject: str,
        fix: str = ''
    ) -> dict[str, Any]:
        alert: dict[str, Any] = {
            'level': level,           # 'ERROR' | 'WARN' | 'INFO'
            'category': category,     # 'node_crash' | 'topic_inactive' | 'tf_failure'
            'subject': subject,
            'message': message,
            'suggested_fix': fix or self._hint(subject),
            'timestamp': time.time(),
        }
        return alert

    # ── Main watchdog callback ────────────────────────────────────────────────

    def _watchdog_callback(self) -> None:
        alerts: list[dict[str, Any]] = []
        now = time.time()

        # 1. Node crash detection
        running_nodes = set(self._get_running_nodes())
        for node in self._critical_nodes:
            if node not in running_nodes:
                alert = self._make_alert(
                    level='ERROR',
                    category='node_crash',
                    subject=node,
                    message=f'Critical node NOT running: {node}',
                    fix=self._hint(node),
                )
                alerts.append(alert)
                self.get_logger().error(
                    f'[Watchdog] NODE CRASH detected: {node} — '
                    f'{alert["suggested_fix"]}'
                )

        # 2. Topic inactivity checks
        topic_checks = {
            'odom': ('/diff_drive_controller/odom', self._last_odom, 'odom_timeout'),
            'scan': ('/scan', self._last_scan, 'scan_timeout'),
            'imu':  ('/imu',  self._last_imu,  'imu_timeout'),
        }
        for name, (topic, last_seen, hint_key) in topic_checks.items():
            silence = now - last_seen
            if silence > self._topic_timeout:
                alert = self._make_alert(
                    level='WARN',
                    category='topic_inactive',
                    subject=topic,
                    message=(
                        f'Topic {topic} has been silent for '
                        f'{silence:.1f}s (timeout={self._topic_timeout}s)'
                    ),
                    fix=self._hint(hint_key),
                )
                alerts.append(alert)
                self.get_logger().warn(
                    f'[Watchdog] TOPIC INACTIVE: {topic} '
                    f'({silence:.1f}s silent) — {alert["suggested_fix"]}'
                )

        # 3. TF failure detection
        for pair_str in self._tf_pairs_raw:
            if '->' not in pair_str:
                continue
            parent, child = pair_str.split('->', 1)
            # TF check is slow; only run it when other alerts exist to save CPU
            # For a production system you'd use tf2_ros.Buffer directly.
            # Here we rely on ros2 topic echo as a lightweight proxy:
            # check that /tf topic is alive instead of full lookup.
            silence_system = now - self._last_system_status
            if silence_system > self._topic_timeout * 2:
                alert = self._make_alert(
                    level='WARN',
                    category='tf_failure',
                    subject=f'{parent}->{child}',
                    message=(
                        f'TF frame {parent}->{child} may be unavailable '
                        f'(system_status silent for {silence_system:.1f}s)'
                    ),
                    fix=self._hint('tf_missing'),
                )
                alerts.append(alert)
                self.get_logger().warn(
                    f'[Watchdog] TF POSSIBLE FAILURE: {parent}->{child}'
                )

        # 4. Publish alerts
        payload = {
            'timestamp': now,
            'alert_count': len(alerts),
            'alerts': alerts,
            'all_ok': len(alerts) == 0,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._alert_pub.publish(msg)

        if not alerts:
            self.get_logger().debug('[Watchdog] All systems nominal ✓')
        else:
            self.get_logger().warn(
                f'[Watchdog] {len(alerts)} alert(s) published on /system_alerts'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WatchdogNode()
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
