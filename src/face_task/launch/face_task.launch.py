"""
face_task.launch.py
====================
Launches only the face-task pipeline nodes.

Included by bringup_sim.launch.py (with a 10 s delay).
Can also be launched standalone on top of a running simulation:

  ros2 launch face_task face_task.launch.py \\
      target_image:=/path/to/photo1.jpg

Nodes started
-------------
  turret_gazebo_bridge   — /turret/pan_deg + /turret/tilt_deg
                           → /turret_controller/commands  (Gazebo, radians)
  face_recognition_node  — InsightFace on /turret_camera/image_raw
  face_task_node         — 21-position boustrophedon scan state machine
  turret_controller_node — dry_run=true in sim (no serial needed)
  face_task_trigger_node — fires on /waypoint_reached WP-2

Topic wiring
------------
  face_task_node  →  /turret/pan_deg + /turret/tilt_deg
  turret_gazebo_bridge  →  /turret_controller/commands  →  Gazebo joint
  Gazebo joint  →  /turret_camera/image_raw
  face_recognition_node  →  /face/match_found  →  face_task_node
  face_task_trigger_node  ←  /waypoint_reached
  face_task_trigger_node  →  /face_task/start
  face_task_node  →  /face_task/complete
  face_task_trigger_node  →  /face_task/done
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    args = [
        DeclareLaunchArgument(
            'target_image',
            default_value='',
            description=(
                'Absolute path to the target face JPEG. '
                'Example: $(ros2 pkg prefix simulation)'
                '/share/simulation/models/images/photo1.jpg'
            ),
        ),
        DeclareLaunchArgument(
            'similarity_threshold',
            default_value='0.35',
            description='ArcFace cosine similarity threshold (0.0–1.0). '
                        'Lower = more permissive. Try 0.30 if matches are missed.',
        ),
        DeclareLaunchArgument(
            'trigger_waypoint_name',
            default_value='WP-2',
            description='Waypoint name that fires the face task '
                        '(must match waypoint_names in waypoints.yaml).',
        ),
        DeclareLaunchArgument(
            'hard_timeout_sec',
            default_value='55.0',
            description='Hard timeout (s) before face task force-exits. '
                        'Keep below 60 to avoid the competition stuck penalty.',
        ),
    ]

    # Converts /turret/pan_deg + /turret/tilt_deg (Float32, degrees)
    # into /turret_controller/commands (Float64MultiArray, radians) for Gazebo.
    turret_gazebo_bridge = Node(
        package='face_task',
        executable='turret_gazebo_bridge',
        name='turret_gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # InsightFace inference on the turret camera feed.
    face_recognition = Node(
        package='face_task',
        executable='face_recognition',
        name='face_recognition_node',
        output='screen',
        parameters=[{
            'target_image_path':    LaunchConfiguration('target_image'),
            'similarity_threshold': LaunchConfiguration('similarity_threshold'),
            'use_sim_time':         True,
        }],
        remappings=[
            # Turret camera publishes here via ros_gz_bridge in sim.
            ('/camera/image_raw', '/turret_camera/image_raw'),
        ],
    )

    # 21-position boustrophedon scan state machine.
    face_task_node = Node(
        package='face_task',
        executable='face_task',
        name='face_task_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Serial bridge to Arduino / ESP32.
    # dry_run=True in sim — actual movement happens via turret_gazebo_bridge.
    turret_controller = Node(
        package='face_task',
        executable='turret_controller',
        name='turret_controller_node',
        output='screen',
        parameters=[{
            'dry_run':      True,
            'use_sim_time': True,
        }],
    )

    # Listens to /waypoint_reached; fires /face_task/start on WP-2.
    # Also zeroes cmd_vel to hold the robot still during the scan.
    face_task_trigger = Node(
        package='face_task',
        executable='face_task_trigger',
        name='face_task_trigger_node',
        output='screen',
        parameters=[{
            'trigger_waypoint_name':   LaunchConfiguration('trigger_waypoint_name'),
            'trigger_waypoint_index':  2,
            'hard_timeout_sec':        LaunchConfiguration('hard_timeout_sec'),
            'stop_vehicle_on_trigger': True,
            'cmd_vel_topic':           '/cmd_vel',
            'use_sim_time':            True,
        }],
    )

    return LaunchDescription(args + [
        turret_gazebo_bridge,
        face_recognition,
        face_task_node,
        turret_controller,
        face_task_trigger,
    ])