"""
monitoring_all.launch.py
========================
Launches all monitoring components from the watchdog_monitor package:
  - system_monitor_node    → /system_status
  - watchdog_node          → /system_alerts
  - waypoint_detector_node → /waypoint_reached, /waypoint_status
  - control_listener_node   (read-only integration)

Usage:
  ros2 launch watchdog_monitor monitoring_all.launch.py

Optional overrides:
  monitor_interval:=2.0
  topic_timeout:=5.0
  arrival_radius:=0.5
  odom_topic:=/diff_drive_controller/odom
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory('watchdog_monitor')
    waypoints_yaml = os.path.join(pkg, 'config', 'waypoints.yaml')

    # ── Arguments ─────────────────────────────────────────────────────────────
    declare_monitor_interval = DeclareLaunchArgument(
        'monitor_interval', default_value='2.0',
        description='System monitor check interval (s)'
    )
    declare_topic_timeout = DeclareLaunchArgument(
        'topic_timeout', default_value='5.0',
        description='Topic inactivity timeout (s)'
    )
    declare_arrival_radius = DeclareLaunchArgument(
        'arrival_radius', default_value='0.5',
        description='Waypoint arrival radius (m)'
    )
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic', default_value='/diff_drive_controller/odom',
        description='Odometry topic for waypoint detection'
    )

    # ── Nodes ─────────────────────────────────────────────────────────────────
    system_monitor_node = Node(
        package='watchdog_monitor',
        executable='system_monitor_node',
        name='system_monitor_node',
        output='screen',
        parameters=[{
            'monitor_interval': LaunchConfiguration('monitor_interval'),
            'expected_nodes': [
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
            ],
        }]
    )

    watchdog_node = Node(
        package='watchdog_monitor',
        executable='watchdog_node',
        name='watchdog_node',
        output='screen',
        parameters=[{
            'topic_timeout': LaunchConfiguration('topic_timeout'),
            'critical_nodes': [
                '/robot_state_publisher',
                '/ekf_filter_node',
                '/slam_toolbox',
                '/lifecycle_manager_localization',
                '/planner_server',
                '/controller_server',
                '/bt_navigator',
                '/behavior_server',
                '/lifecycle_manager_navigation',
                '/diff_drive_controller',
                '/joint_state_broadcaster',
                '/lane_costmap',
            ],
            'tf_pairs': [
                'odom->base_link',
                'map->odom',
                'base_link->laser',
            ],
        }]
    )

    waypoint_detector_node = Node(
        package='watchdog_monitor',
        executable='waypoint_detector_node',
        name='waypoint_detector_node',
        output='screen',
        parameters=[
            waypoints_yaml,
            {
                'odom_topic': LaunchConfiguration('odom_topic'),
                'arrival_radius': LaunchConfiguration('arrival_radius'),
            }
        ]
    )

    control_listener_node = Node(
        package='watchdog_monitor',
        executable='control_listener_node',
        name='control_listener_node',
        output='screen',
    )

    return LaunchDescription([
        declare_monitor_interval,
        declare_topic_timeout,
        declare_arrival_radius,
        declare_odom_topic,
        system_monitor_node,
        watchdog_node,
        waypoint_detector_node,
        control_listener_node,
    ])
