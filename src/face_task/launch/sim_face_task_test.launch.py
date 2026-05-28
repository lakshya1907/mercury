#!/usr/bin/env python3
"""
sim_face_task_test.launch.py
============================
Single-command launch for the full simulation face-detection test.

What happens (in order)
-----------------------
  t=0s   Gazebo + robot spawn + ros2_control controllers start
         (via bringup_sim → simulation.launch + bringup_base.launch)

  t=0s   waypoint_detector_node starts (monitors /diff_drive_controller/odom
         and publishes /waypoint_reached when robot enters WP radius)

  t=10s  Face-task pipeline nodes start:
           • turret_gazebo_bridge  — /turret/pan_deg + /turret/tilt_deg
                                     → /turret_controller/commands (Gazebo)
           • face_recognition_node — InsightFace on /turret_camera/image_raw
           • face_task_node        — 21-position boustrophedon scan SM
           • turret_controller_node — dry_run=true (no serial in sim)
           • face_task_trigger_node — fires on WP-2 /waypoint_reached event

  t=15s  sim_waypoint_nav_node starts:
           • Sends Nav2 NavigateToPose goal → WP-1 (2.0, 0.0) [optional]
           • Then → WP-2 (-6.75, -50.0)  — face photo gallery
           • On arrival publishes /waypoint_reached (WP-2 JSON)
           • Holds robot still (zero cmd_vel) while face detection runs
           • Waits for /face_task/done then stops (robot stays at WP-2)

  face detection flow (once triggered)
  -------------------------------------
  face_task_trigger_node
      receives /waypoint_reached {name:"WP-2"}
      → publishes /face_task/start = True
      → keeps publishing zero cmd_vel

  face_task_node (SCANNING state)
      iterates 21 pan×tilt positions
      → /turret/pan_deg, /turret/tilt_deg at each position
      → triggers /face/capture_request

  turret_gazebo_bridge
      /turret/pan_deg + /turret/tilt_deg
      → /turret_controller/commands (Float64MultiArray, radians)
      → Gazebo joint controller moves camera

  face_recognition_node
      /face/capture_request triggers analysis of /turret_camera/image_raw
      → /face/match_found, /face/horizontal_error, /face/vertical_error

  face_task_node (FINE_TUNE + FIRE states)
      centres turret on face, fires laser, publishes /face_task/complete

  face_task_trigger_node
      /face_task/complete → publishes /face_task/done = True

  sim_waypoint_nav_node
      /face_task/done → stops sending cmd_vel, transitions to DONE
      robot remains stationary at WP-2

Usage
-----
  # Minimal (uses photo1.jpg as target face)
  ros2 launch face_task sim_face_task_test.launch.py \\
      target_image:=$(ros2 pkg prefix simulation)/share/simulation/models/images/photo1.jpg

  # Choose target face
  ros2 launch face_task sim_face_task_test.launch.py \\
      target_image:=/path/to/photo3.jpg \\
      similarity_threshold:=0.30

  # Skip WP-1, go straight to face zone
  ros2 launch face_task sim_face_task_test.launch.py \\
      target_image:=/path/to/photo1.jpg \\
      skip_wp1:=true

  # Headless (no Gazebo GUI)
  GZ_HEADLESS=1 ros2 launch face_task sim_face_task_test.launch.py \\
      target_image:=/path/to/photo1.jpg

Topics to monitor
-----------------
  ros2 topic echo /face_task/state          # trigger node state
  ros2 topic echo /face/match_found         # face match result per capture
  ros2 topic echo /face/best_similarity     # similarity score
  ros2 topic echo /turret/pan_deg           # turret movement
  ros2 topic echo /turret/tilt_deg
  ros2 topic echo /face_task/complete       # face_task_node finished
  ros2 topic echo /face_task/done           # trigger node finished
  ros2 topic echo /waypoint_status          # waypoint progress
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
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
                'Absolute path to the target face JPEG. '
                'Use one of the pre-installed sim images: '
                '$(ros2 pkg prefix simulation)/share/simulation/models/images/photo1.jpg'
            ),
        ),
        DeclareLaunchArgument(
            'similarity_threshold',
            default_value='0.35',
            description=(
                'ArcFace cosine similarity threshold (0.0-1.0). '
                'Lower = more permissive. Try 0.30 if matches are missed.'
            ),
        ),
        DeclareLaunchArgument(
            'trigger_waypoint_name',
            default_value='WP-2',
            description='Waypoint name that fires the face task.',
        ),
        DeclareLaunchArgument(
            'hard_timeout_sec',
            default_value='55.0',
            description='Hard timeout (s) before face task force-exits.',
        ),
        DeclareLaunchArgument(
            'skip_wp1',
            default_value='false',
            description='Skip WP-1 and drive straight to the face-photo zone.',
        ),
        DeclareLaunchArgument(
            'arrival_radius',
            default_value='1.5',
            description='Metres — distance to count as arrived at a waypoint.',
        ),
        # WP-2 coordinates (centre of photo gallery in mercury.sdf)
        DeclareLaunchArgument(
            'wp2_x', default_value='-6.75',
            description='WP-2 x coordinate (face gallery centre).',
        ),
        DeclareLaunchArgument(
            'wp2_y', default_value='-50.0',
            description='WP-2 y coordinate (face gallery row).',
        ),
    ]

    # ── 1. Full simulation bringup ────────────────────────────────────────────
    #    Starts: Gazebo, robot spawn, ros2_control, ros_gz_bridge, Nav2, RViz
    bringup_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bringup'), 'launch', 'bringup_sim.launch.py',
            ])
        )
    )

    # ── 2. Waypoint detector (starts immediately, listens to odom) ────────────
    waypoint_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('watchdog_monitor'),
                'launch', 'waypoint_manager.launch.py',
            ])
        ),
        launch_arguments={
            'arrival_radius': LaunchConfiguration('arrival_radius'),
        }.items(),
    )

    # ── 3. Face-task pipeline (delayed 10s — wait for Gazebo + controllers) ───

    # 3a. Turret Gazebo bridge: converts degree commands → Gazebo joint radians
    turret_gazebo_bridge = Node(
        package='perception',
        executable='turret_gazebo_bridge',
        name='turret_gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 3b. Face recognition: InsightFace on turret camera feed
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
            # The turret camera publishes here in sim (from ros_gz_bridge)
            ('/camera/image_raw', '/turret_camera/image_raw'),
        ],
    )

    # 3c. Face task state machine: 21-position boustrophedon grid scan
    face_task_node = Node(
        package='face_task',
        executable='face_task',
        name='face_task_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 3d. Turret controller: dry_run in sim (movement via bridge above)
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

    # 3e. Face task trigger: listens to /waypoint_reached, fires on WP-2
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

    face_task_pipeline = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[sim_face_task_test] Starting face-task pipeline nodes...'),
            turret_gazebo_bridge,
            face_recognition,
            face_task_node,
            turret_controller,
            face_task_trigger,
        ],
    )

    # ── 4. Sim waypoint nav (delayed 15s — wait for Nav2 to be ready) ─────────
    #    Drives robot: WP-1 → WP-2, then holds still during face task
    sim_waypoint_nav = Node(
        package='face_task',
        executable='sim_waypoint_nav',
        name='sim_waypoint_nav_node',
        output='screen',
        parameters=[{
            # WP-1: gentle on-track warmup point
            'wp1_x':                2.0,
            'wp1_y':                0.0,
            # WP-2: face photo gallery (centre of 6 photos in mercury.sdf)
            'wp2_x':                LaunchConfiguration('wp2_x'),
            'wp2_y':                LaunchConfiguration('wp2_y'),
            # WP-3: exit point (robot stays at WP-2 per your requirement)
            'wp3_x':                2.0,
            'wp3_y':               -55.0,
            'arrival_radius':       LaunchConfiguration('arrival_radius'),
            'navigate_to_wp3':      False,   # stay at WP-2 after face task
            'nav_startup_delay_sec': 15.0,   # wait for Nav2
            'skip_wp1':             LaunchConfiguration('skip_wp1'),
            'use_sim_time':         True,
        }],
    )

    sim_nav_delayed = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg=(
                '[sim_face_task_test] Starting sim_waypoint_nav_node. '
                'Robot will navigate to WP-2 and trigger face detection.'
            )),
            sim_waypoint_nav,
        ],
    )

    return LaunchDescription(args + [
        LogInfo(msg='=== sim_face_task_test: launching simulation + face detection pipeline ==='),
        bringup_sim,
        waypoint_manager,
        face_task_pipeline,
        sim_nav_delayed,
    ])
