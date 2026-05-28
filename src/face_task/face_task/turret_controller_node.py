#!/usr/bin/env python3
"""
turret_controller_node.py
=========================
Hardware bridge node. Converts ROS2 servo angle commands to PWM signals
sent over serial to an Arduino (or ESP32).

Serial protocol (simple ASCII, one command per line):
    P<angle>    -- set pan servo   e.g. "P95.5\n"
    T<angle>    -- set tilt servo  e.g. "T112.0\n"
    L1          -- laser ON
    L0          -- laser OFF

Subscribes:
    /turret/pan_deg    (std_msgs/Float32)  servo angle in degrees 0-180
    /turret/tilt_deg   (std_msgs/Float32)  servo angle in degrees 0-180
    /laser/fire        (std_msgs/Bool)

Parameters:
    serial_port    default: /dev/ttyUSB1  (change to your Arduino port)
    baud_rate      default: 115200
    dry_run        default: true   (if true: log commands but don't open serial)
                                   Set to false on real hardware
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import time


class TurretControllerNode(Node):

    def __init__(self):
        super().__init__('turret_controller_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate',   115200)
        self.declare_parameter('dry_run',     True)

        port     = self.get_parameter('serial_port').value
        baud     = self.get_parameter('baud_rate').value
        self._dry = self.get_parameter('dry_run').value

        # ── Serial connection ─────────────────────────────────────────────────
        self._serial = None
        if not self._dry:
            try:
                import serial
                self._serial = serial.Serial(port, baud, timeout=0.1)
                time.sleep(2.0)  # wait for Arduino reset
                self.get_logger().info(f'Serial connected: {port} @ {baud}')
            except Exception as e:
                self.get_logger().error(f'Serial open failed: {e}')
                self.get_logger().warn('Falling back to dry_run mode.')
                self._dry = True
        else:
            self.get_logger().warn(
                f'dry_run=True — serial commands will be LOGGED ONLY, not sent. '
                f'Set dry_run:=false on real hardware.')

        # ── State ─────────────────────────────────────────────────────────────
        self._last_pan   = None
        self._last_tilt  = None
        self._laser_on   = False

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Float32, '/turret/pan_deg',  self._pan_cb,   10)
        self.create_subscription(Float32, '/turret/tilt_deg', self._tilt_cb,  10)
        self.create_subscription(Bool,    '/laser/fire',      self._laser_cb, 10)

        self.get_logger().info('TurretControllerNode ready.')

    # ──────────────────────────────────────────────────────────────────────────
    def _pan_cb(self, msg: Float32):
        angle = self._clamp(msg.data, 0.0, 180.0)
        if angle != self._last_pan:
            self._send(f'P{angle:.1f}')
            self._last_pan = angle

    def _tilt_cb(self, msg: Float32):
        angle = self._clamp(msg.data, 0.0, 180.0)
        if angle != self._last_tilt:
            self._send(f'T{angle:.1f}')
            self._last_tilt = angle

    def _laser_cb(self, msg: Bool):
        cmd = 'L1' if msg.data else 'L0'
        if msg.data != self._laser_on:
            self._send(cmd)
            self._laser_on = msg.data

    # ──────────────────────────────────────────────────────────────────────────
    def _send(self, cmd: str):
        """Send ASCII command to Arduino over serial."""
        line = cmd + '\n'
        if self._dry:
            self.get_logger().info(f'[DRY-RUN] serial → "{cmd}"')
        else:
            try:
                self._serial.write(line.encode('ascii'))
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')

    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))

    def destroy_node(self):
        if self._serial and self._serial.is_open:
            self._send('L0')  # laser off on shutdown
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TurretControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
