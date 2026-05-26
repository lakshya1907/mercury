from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    args = [
        DeclareLaunchArgument('target_image',            default_value='',             description='Path to target face image'),
        DeclareLaunchArgument('similarity_threshold',    default_value='0.35',         description='ArcFace cosine similarity threshold'),
        DeclareLaunchArgument('trigger_waypoint_name',   default_value='WP-2',         description='Waypoint name that triggers face task'),
        DeclareLaunchArgument('trigger_waypoint_index',  default_value='2',            description='1-based waypoint index fallback'),
        DeclareLaunchArgument('hard_timeout_sec',        default_value='55.0',         description='Hard timeout for face task (sec)'),
        DeclareLaunchArgument('stop_vehicle_on_trigger', default_value='true',         description='Publish zero cmd_vel while scanning'),
        DeclareLaunchArgument('cmd_vel_topic',           default_value='/cmd_vel',     description='cmd_vel topic to zero while scanning'),
        DeclareLaunchArgument('camera_topic',            default_value='/camera/image_raw', description='Camera image topic'),
        DeclareLaunchArgument('dry_run',                 default_value='true',         description='Log serial cmds only, do not send'),
        DeclareLaunchArgument('serial_port',             default_value='/dev/ttyUSB1', description='Arduino/ESP32 serial port'),
        DeclareLaunchArgument('baud_rate',               default_value='115200',       description='Serial baud rate'),
    ]

    face_recognition = Node(
        package='face_task', executable='face_recognition',
        name='face_recognition_node', output='screen',
        parameters=[{
            'target_image_path':    LaunchConfiguration('target_image'),
            'similarity_threshold': LaunchConfiguration('similarity_threshold'),
        }],
        remappings=[('/camera/image_raw', LaunchConfiguration('camera_topic'))],
    )

    face_task_node = Node(
        package='face_task', executable='face_task',
        name='face_task_node', output='screen',
    )

    turret_controller = Node(
        package='face_task', executable='turret_controller',
        name='turret_controller_node', output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate':   LaunchConfiguration('baud_rate'),
            'dry_run':     LaunchConfiguration('dry_run'),
        }],
    )

    face_task_trigger = Node(
        package='face_task', executable='face_task_trigger',
        name='face_task_trigger_node', output='screen',
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
        face_task_node,
        turret_controller,
        face_task_trigger,
    ])
