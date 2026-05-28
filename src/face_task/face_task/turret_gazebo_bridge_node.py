#!/usr/bin/env python3
"""
turret_gazebo_bridge.py
=======================
Converts /turret/pan_deg and /turret/tilt_deg (in degrees, robot-frame)
to /turret_controller/commands (Float64MultiArray, radians, Gazebo joint frame).

Joint definitions from URDF:
  pan_joint      — axis Z (0 0 1) — parent: base_link
                   limit: [-3.14, 3.14] rad
                   0 rad = facing forward (+X)
                   positive = counter-clockwise (left) when viewed from above

  tilt_base_joint — axis Y (0 1 0) — parent: pan_link
                   limit: [-1.57, 1.57] rad
                   0 rad = level (horizontal)
                   positive = tilt up, negative = tilt down

Convention used by face_task_node:
  pan_deg:  0 = forward, positive = left, negative = right  (matches Z-up CCW)
  tilt_deg: 0 = level,   positive = up,   negative = down

So the conversion is simply:
  pan_joint_rad  = pan_deg  * pi/180        (direct, same sign convention)
  tilt_joint_rad = tilt_deg * pi/180        (direct, same sign convention)

Controllers are ordered [pan_joint, tilt_base_joint] per controllers.yaml.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray


class TurretGazeboBridge(Node):

    def __init__(self):
        super().__init__('turret_gazebo_bridge')

        # Current commanded angles in degrees (robot frame)
        self._pan_deg  = 0.0   # 0 = straight forward
        self._tilt_deg = 0.0   # 0 = level

        # Joint limits from URDF (used for clamping before publish)
        self._pan_min  = -170.0   # degrees (-3.14 rad with small margin)
        self._pan_max  =  170.0
        self._tilt_min = -80.0    # degrees (-1.57 rad with small margin)
        self._tilt_max =  80.0

        self.create_subscription(Float32, '/turret/pan_deg',  self._pan_cb,  10)
        self.create_subscription(Float32, '/turret/tilt_deg', self._tilt_cb, 10)

        self._pub = self.create_publisher(
            Float64MultiArray, '/turret_controller/commands', 10)

        # Publish home position on startup so turret starts level/forward
        self._publish_joints()

        self.get_logger().info(
            'TurretGazeboBridge ready. '
            'pan=0°→forward, tilt=0°→level, tilt+→up, pan+→left')

    def _pan_cb(self, msg: Float32) -> None:
        self._pan_deg = max(self._pan_min, min(self._pan_max, msg.data))
        self._publish_joints()

    def _tilt_cb(self, msg: Float32) -> None:
        self._tilt_deg = max(self._tilt_min, min(self._tilt_max, msg.data))
        self._publish_joints()

    def _publish_joints(self) -> None:
        pan_rad  = math.radians(self._pan_deg)
        tilt_rad = math.radians(self._tilt_deg)

        msg = Float64MultiArray()
        # Order must match controllers.yaml: [pan_joint, tilt_base_joint]
        msg.data = [pan_rad, tilt_rad]
        self._pub.publish(msg)

        self.get_logger().info(
            f'Turret → pan={self._pan_deg:+.1f}° ({pan_rad:+.3f}rad)  '
            f'tilt={self._tilt_deg:+.1f}° ({tilt_rad:+.3f}rad)')


def main():
    rclpy.init()
    node = TurretGazeboBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
