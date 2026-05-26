"""
face_task.launch.py
====================
Launches the complete WP-2 face-detection pipeline:

  1. face_recognition_node   — camera + InsightFace inference
  2. face_task_node          — 18-position scan state machine
  3. turret_controller_node  — serial bridge to Arduino/ESP32
  4. face_task_trigger_node  — listens to /waypoint_reached,
                               fires /face_task/start on WP-2

Usage
-----
  # Simulation / dry-run (no real serial port)
  ros2 launch perception face_task.launch.py \\
      target_image:=/path/to/target.jpg

  # Real hardware
  ros2 launch perception face_task.launch.py \\
      target_image:=/path/to/target.jpg \\
      dry_run:=false \\
      serial_port:=/dev/ttyUSB1

All existing nodes (waypoint_detector_node, navigation stack, etc.) continue
running unchanged — this launch adds four new nodes alongside them.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ── Arguments (all have safe defaults so the launch works out-of-box) ────
    args = [
        DeclareLaunchArgument(
            'target_image',
            default_value='',
            description='Absolute path to the pre-given target face image (JPEG/PNG)'),

        DeclareLaunchArgument(
            'similarity_threshold',
            default_value='0.35',
            description='ArcFace cosine similarity threshold (0–1). '
                        'Lower = more permissive.'),

        DeclareLaunchArgument(
            'trigger_waypoint_name',
            default_value='WP-2',
            description='Name of the waypoint that triggers the face task '
                        '(must match waypoint_names in waypoints.yaml)'),

        DeclareLaunchArgument(
            'trigger_waypoint_index',
            default_value='2',
            description='1-based waypoint index fallback trigger'),

        DeclareLaunchArgument(
            'hard_timeout_sec',
            default_value='55.0',
            description='Hard timeout for the face task in seconds. '
                        'Set < 60 to avoid the competition "stuck" penalty.'),

        DeclareLaunchArgument(
            'dry_run',
            default_value='true',
            description='If true, serial commands are logged but not sent. '
                        'Set false on real hardware.'),

        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for turret Arduino/ESP32'),

        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Serial baud rate'),

        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/image_raw',
            description='Camera image topic'),

        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='cmd_vel topic to zero-out while the vehicle is scanning'),

        DeclareLaunchArgument(
            'stop_vehicle_on_trigger',
            default_value='true',
            description='Publish zero cmd_vel while face scan is running'),
    ]

    # ── Nodes ─────────────────────────────────────────────────────────────────

    face_recognition = Node(
        package='perception',
        executable='face_recognition',
        name='face_recognition_node',
        output='screen',
        parameters=[{
            'target_image_path':    LaunchConfiguration('target_image'),
            'similarity_threshold': LaunchConfiguration('similarity_threshold'),
        }],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('camera_topic')),
        ],
    )

    face_task = Node(
        package='perception',
        executable='face_task',
        name='face_task_node',
        output='screen',
        # All scan-grid parameters use defaults from the node.
        # Override here if needed:
        # parameters=[{'settle_time_sec': 0.4}],
    )

    turret_controller = Node(
        package='perception',
        executable='turret_controller',
        name='turret_controller_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate':   LaunchConfiguration('baud_rate'),
            'dry_run':     LaunchConfiguration('dry_run'),
        }],
    )

    face_task_trigger = Node(
        package='perception',
        executable='face_task_trigger',
        name='face_task_trigger_node',
        output='screen',
        parameters=[{
            'trigger_waypoint_name':   LaunchConfiguration('trigger_waypoint_name'),
            'trigger_waypoint_index':  LaunchConfiguration('trigger_waypoint_index'),
            'hard_timeout_sec':        LaunchConfiguration('hard_timeout_sec'),
            'stop_vehicle_on_trigger': LaunchConfiguration('stop_vehicle_on_trigger'),
            'cmd_vel_topic':           LaunchConfiguration('cmd_vel_topic'),
        }],
    )

    return LaunchDescription(args + [
        face_recognition,
        face_task,
        turret_controller,
        face_task_trigger,
    ])
