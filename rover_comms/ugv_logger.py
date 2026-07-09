#!/usr/bin/env python3
"""
ugv_logger.py  —  ROS 2 logger node
Subscribes to all topics and writes to ~/robot_logs/*.json
Also writes video frames to ~/robot_logs/video_frames/ for the bridge to pick up.

Run with:
    python3 ugv_logger.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu, NavSatFix, JointState, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from action_msgs.msg import GoalStatusArray
from rclpy.qos import qos_profile_sensor_data
import json
import os
import cv2
import numpy as np
from datetime import datetime
import psutil
import time


class LoggerNode(Node):

    def __init__(self):
        super().__init__('logger_node')

        self.base_path  = os.path.expanduser("~/robot_logs")
        self.video_path = os.path.join(self.base_path, "video_frames")
        self.turret_video_path = os.path.join(self.base_path, "video_frames_turret")
        os.makedirs(self.base_path,  exist_ok=True)
        os.makedirs(self.video_path, exist_ok=True)
        os.makedirs(self.turret_video_path, exist_ok=True)

        self.system_file   = open(self._file("system_log.json"),          "a")
        self.alert_file    = open(self._file("alerts_log.json"),           "a")
        self.state_file    = open(self._file("state_log.json"),            "a")
        self.control_file  = open(self._file("control_log.json"),          "a")
        self.nav_file      = open(self._file("navigation_log.json"),       "a")
        self.resource_file = open(self._file("system_resource_log.json"),  "a")
        self.encoder_file  = open(self._file("encoder_log.json"),          "a")
        self.power_file    = open(self._file("power_log.json"),            "a")

        self._frame_id      = 0
        self._last_frame_t  = 0.0
        self._frame_interval = 1.0 / 15   # 15 fps max to avoid flooding disk

        self._turret_frame_id     = 0
        self._last_turret_frame_t = 0.0
        self._turret_frame_interval = 1.0 / 15

        # Rate limiting for high-frequency topics (avoid disk/terminal flood)
        self._last_log_t = {"imu": 0.0, "gps": 0.0, "odom": 0.0, "encoder": 0.0,
                             "cmd_vel": 0.0, "cmd_vel_nav": 0.0}
        self._log_interval = {
            "imu": 1.0 / 20,      # 20 Hz max (was unthrottled at sensor rate, often 100-200Hz)
            "gps": 1.0 / 5,       # 5 Hz — GPS doesn't update faster than this anyway
            "odom": 1.0 / 20,     # 20 Hz
            "encoder": 1.0 / 20,  # 20 Hz
            "cmd_vel": 1.0 / 10,  # 10 Hz
            "cmd_vel_nav": 1.0 / 10,
        }

        self.get_logger().info("Logger node started...")

        self.create_subscription(String,         '/system_status',  self.system_cb,     10)
        self.create_subscription(String,         '/system_alerts',  self.alert_cb,      10)

        self.create_subscription(Imu,            '/imu',            self.imu_cb,        qos_profile_sensor_data)
        self.create_subscription(NavSatFix,      '/gps',            self.gps_cb,        qos_profile_sensor_data)
        self.create_subscription(Odometry,       '/diff_drive_controller/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(JointState,     '/joint_states',   self.encoder_cb,    qos_profile_sensor_data)
        self.create_subscription(Image,          '/camera/image_raw', self.camera_cb,   qos_profile_sensor_data)
        self.create_subscription(Image,          '/turret_camera/image_raw', self.turret_camera_cb, qos_profile_sensor_data)

        self.create_subscription(Twist,          '/cmd_vel',        self.cmd_cb,        10)
        self.create_subscription(Twist,          '/cmd_vel_nav',    self.cmd_nav_cb,    10)

        self.create_subscription(PoseStamped,    '/goal_pose',      self.goal_cb,       10)
        self.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status', self.nav_status_cb, 10)

        self.create_timer(2.0, self.system_resource_cb)
        self.create_timer(5.0, self.power_cb)

    def _file(self, name):
        return os.path.join(self.base_path, name)

    def _time(self):
        return datetime.utcnow().isoformat()

    def _write(self, file, data, label, quiet=False):
        json_line = json.dumps(data)
        file.write(json_line + "\n")
        file.flush()
        if not quiet:
            self.get_logger().info(f"{label}: {json_line}")

    def _should_log(self, key: str) -> bool:
        """Returns True if enough time has passed since last log for this key."""
        now = time.time()
        if now - self._last_log_t[key] < self._log_interval[key]:
            return False
        self._last_log_t[key] = now
        return True

    # ── Existing callbacks (unchanged) ────────────────────────────────────────
    def system_cb(self, msg):
        self._write(self.system_file, {
            "time": self._time(), "system_status": msg.data
        }, "SYSTEM")

    def alert_cb(self, msg):
        self._write(self.alert_file, {
            "time": self._time(), "alert": msg.data
        }, "ALERT")

    def imu_cb(self, msg):
        if not self._should_log("imu"):
            return
        self._write(self.state_file, {
            "time": self._time(),
            "imu": {
                "orientation":         [float(msg.orientation.x), float(msg.orientation.y),
                                        float(msg.orientation.z), float(msg.orientation.w)],
                "angular_velocity":    [float(msg.angular_velocity.x), float(msg.angular_velocity.y),
                                        float(msg.angular_velocity.z)],
                "linear_acceleration": [float(msg.linear_acceleration.x), float(msg.linear_acceleration.y),
                                        float(msg.linear_acceleration.z)],
            }
        }, "IMU", quiet=True)

    def gps_cb(self, msg):
        if not self._should_log("gps"):
            return
        self._write(self.state_file, {
            "time": self._time(),
            "gps": {
                "lat": float(msg.latitude),
                "lon": float(msg.longitude),
                "alt": float(msg.altitude),
            }
        }, "GPS", quiet=True)

    def odom_cb(self, msg):
        if not self._should_log("odom"):
            return
        self._write(self.state_file, {
            "time": self._time(),
            "odom": {
                "pos": [float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)],
                "vel": [float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z)],
            }
        }, "ODOM", quiet=True)

    def encoder_cb(self, msg):
        if not self._should_log("encoder"):
            return
        self._write(self.encoder_file, {
            "time": self._time(),
            "encoder": {
                "names":    list(msg.name),
                "position": [float(x) for x in msg.position],
                "velocity": [float(x) for x in msg.velocity],
            }
        }, "ENCODER", quiet=True)

    def cmd_cb(self, msg):
        if not self._should_log("cmd_vel"):
            return
        self._write(self.control_file, {
            "time": self._time(),
            "cmd_vel": {"linear": float(msg.linear.x), "angular": float(msg.angular.z)}
        }, "CMD", quiet=True)

    def cmd_nav_cb(self, msg):
        if not self._should_log("cmd_vel_nav"):
            return
        self._write(self.control_file, {
            "time": self._time(),
            "cmd_vel_nav": {"linear": float(msg.linear.x), "angular": float(msg.angular.z)}
        }, "CMD_NAV", quiet=True)

    def goal_cb(self, msg):
        self._write(self.nav_file, {
            "time": self._time(),
            "goal": {"x": float(msg.pose.position.x), "y": float(msg.pose.position.y)}
        }, "GOAL")

    def nav_status_cb(self, msg):
        statuses = [{"status": int(s.status),
                     "goal_id": [int(x) for x in s.goal_info.goal_id.uuid]}
                    for s in msg.status_list]
        self._write(self.nav_file, {
            "time": self._time(), "nav_status": statuses
        }, "NAV")

    def system_resource_cb(self):
        self._write(self.resource_file, {
            "time":   self._time(),
            "cpu":    float(psutil.cpu_percent()),
            "memory": float(psutil.virtual_memory().percent),
        }, "SYS_RESOURCE", quiet=True)

    def power_cb(self):
        self._write(self.power_file, {
            "time": self._time(), "battery": "N/A", "voltage": "N/A"
        }, "POWER")

    # ── Video frame writer (new) ──────────────────────────────────────────────
    def camera_cb(self, msg: Image):
        now = time.time()
        if now - self._last_frame_t < self._frame_interval:
            return   # rate-limit to 15 fps
        self._last_frame_t = now
        self._frame_id += 1

        try:
            ch    = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding, 3)
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, ch)
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if not ok:
                return

            # Atomic write: write to .tmp then rename so the bridge never
            # reads a half-written JPEG (which would decode as a black frame).
            fname = os.path.join(self.video_path, f"frame_{self._frame_id:08d}.jpg")
            tmp   = fname + ".tmp"
            with open(tmp, "wb") as f:
                f.write(buf.tobytes())
            os.replace(tmp, fname)
        except Exception as e:
            self.get_logger().warning(f"Camera frame write failed: {e}")


    # ── Video frame writer — turret/face camera (new) ─────────────────────────
    def turret_camera_cb(self, msg: Image):
        now = time.time()
        if now - self._last_turret_frame_t < self._turret_frame_interval:
            return
        self._last_turret_frame_t = now
        self._turret_frame_id += 1

        try:
            ch    = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding, 3)
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, ch)
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if not ok:
                return

            fname = os.path.join(self.turret_video_path, f"frame_{self._turret_frame_id:08d}.jpg")
            tmp   = fname + ".tmp"
            with open(tmp, "wb") as f:
                f.write(buf.tobytes())
            os.replace(tmp, fname)
        except Exception as e:
            self.get_logger().warning(f"Turret camera frame write failed: {e}")


def main():
    rclpy.init()
    node = LoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()