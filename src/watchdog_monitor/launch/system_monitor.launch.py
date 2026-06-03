from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    declare_interval_arg = DeclareLaunchArgument(
        'monitor_interval',
        default_value='2.0',
        description='Seconds between each health check cycle'
    )

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
            ]
        }]
    )

    return LaunchDescription([
        declare_interval_arg,
        system_monitor_node,
    ])
