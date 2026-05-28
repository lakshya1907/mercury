"""
face_task_sim.launch.py
========================
Launches the complete simulation + face_task pipeline in one command.

What this starts
----------------
  1. bringup_sim        Gazebo world + robot spawn + ros2_control controllers
                        + ros_gz_bridge (includes /turret_camera/image_raw)
  2. bringup_base       description, localization, Nav2, perception, RViz
  3. waypoint_manager   waypoint_detector_node (reads waypoints.yaml)
  4. turret_gazebo_bridge  converts /turret/pan_deg + /turret/tilt_deg
                           → /turret_controller/commands  (Float64MultiArray, radians)
                           so the Gazebo pan_joint / tilt_base_joint actually move
  5. face_recognition_node  InsightFace on /turret_camera/image_raw
  6. face_task_node          21-position boustrophedon scan state machine
  7. turret_controller_node  dry_run=true (serial not needed in sim)
  8. face_task_trigger_node  fires on /waypoint_reached WP-2

Topic wiring in sim
-------------------
  face_task_node  →  /turret/pan_deg        →  turret_gazebo_bridge
  face_task_node  →  /turret/tilt_deg       →  turret_gazebo_bridge
  turret_gazebo_bridge  →  /turret_controller/commands  →  Gazebo joint
  Gazebo joint moves  →  /turret_camera/image_raw
  /turret_camera/image_raw  →  face_recognition_node
  face_recognition_node  →  /face/match_found  →  face_task_node

Usage
-----
  # Minimal — uses photo1.jpg as target
  ros2 launch face_task face_task_sim.launch.py \\
      target_image:=$(ros2 pkg prefix simulation)/share/simulation/models/images/photo1.jpg

  # Full explicit
  ros2 launch face_task face_task_sim.launch.py \\
      target_image:=/home/<user>/mercury/install/simulation/share/simulation/models/images/photo3.jpg \\
      similarity_threshold:=0.30 \\
      hard_timeout_sec:=55.0
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # ── Launch arguments ──────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'target_image',
            default_value='',
            description=(
                'Absolute path to the target face image. '
                'Use one of the photo*.jpg files already in the sim world: '
                '$(ros2 pkg prefix simulation)/share/simulation/models/images/photo1.jpg'
            ),
        ),
        DeclareLaunchArgument(
            'similarity_threshold',
            default_value='0.35',
            description='ArcFace cosine similarity threshold (0.0-1.0). '
                        'Lower = more permissive. Try 0.30 if detections are missed.',
        ),
        DeclareLaunchArgument(
            'trigger_waypoint_name',
            default_value='WP-2',
            description='Waypoint name that triggers the face task '
                        '(must match waypoint_names in waypoints.yaml)',
        ),
        DeclareLaunchArgument(
            'trigger_waypoint_index',
            default_value='2',
            description='1-based index fallback if name matching fails',
        ),
        DeclareLaunchArgument(
            'hard_timeout_sec',
            default_value='55.0',
            description='Hard timeout for the face task in seconds. '
                        'Keep below 60 to avoid the competition stuck penalty.',
        ),
    ]

    # ── 1. Full sim bringup (Gazebo + robot + ros2_control + bridges) ─────────
    bringup_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bringup'), 'launch', 'bringup_sim.launch.py',
            ])
        )
    )

    # ── 2. Waypoint detector ──────────────────────────────────────────────────
    waypoint_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('watchdog_monitor'),
                'launch', 'waypoint_manager.launch.py',
            ])
        )
    )

    # ── 3-7. Face task nodes (delayed 10s to let Gazebo fully start) ──────────
    #
    # turret_gazebo_bridge (from perception package):
    #   /turret/pan_deg  (Float32, degrees)  →  /turret_controller/commands
    #   /turret/tilt_deg (Float32, degrees)     (Float64MultiArray, radians)
    #   This is what actually moves the camera in Gazebo.
    #
    turret_gazebo_bridge = Node(
        package='perception',
        executable='turret_gazebo_bridge',
        name='turret_gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # face_recognition: remapped to turret camera, not front-facing camera
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
            # Sim publishes from tilt_link camera on this topic
            ('/camera/image_raw', '/turret_camera/image_raw'),
        ],
    )

    # face_task_node: runs the 21-position boustrophedon scan
    face_task_node = Node(
        package='face_task',
        executable='face_task',
        name='face_task_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # turret_controller: dry_run in sim — actual movement via bridge above
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

    # face_task_trigger: listens to /waypoint_reached, fires on WP-2
    face_task_trigger = Node(
        package='face_task',
        executable='face_task_trigger',
        name='face_task_trigger_node',
        output='screen',
        parameters=[{
            'trigger_waypoint_name':   LaunchConfiguration('trigger_waypoint_name'),
            'trigger_waypoint_index':  LaunchConfiguration('trigger_waypoint_index'),
            'hard_timeout_sec':        LaunchConfiguration('hard_timeout_sec'),
            'stop_vehicle_on_trigger': True,
            'cmd_vel_topic':           '/cmd_vel',
            'use_sim_time':            True,
        }],
    )

    face_task_group = TimerAction(
        period=10.0,   # wait for Gazebo + controllers to fully initialise
        actions=[
            turret_gazebo_bridge,
            face_recognition,
            face_task_node,
            turret_controller,
            face_task_trigger,
        ],
    )

    return LaunchDescription(args + [
        bringup_sim,
        waypoint_manager,
        face_task_group,
    ])
