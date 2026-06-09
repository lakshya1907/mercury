from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declare_xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        description='Path to the xacro file'
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('description'),
                'launch',
                'description.launch.py'
            ])
        ),
        launch_arguments={
            'xacro_file': LaunchConfiguration('xacro_file')
        }.items()
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('localization'),
                'launch',
                'localization.launch.py'
            ])
        )
    )

    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('planning'),
                'launch',
                'planning.launch.py'
            ])
        )
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('perception'),
                'launch',
                'perception.launch.py'
            ])
        )
    )

    # ── Lane costmap node ──────────────────────────────────────────────────
    # Projects camera lane boundaries into a persistent OccupancyGrid
    # (/perception/road_costmap, map frame) so Nav2's global planner
    # treats outside-lane areas as lethal obstacles.
    # Parameters below must match the camera URDF sensor config and the
    # global_costmap.yaml map extent / origin.
    

    # ── Goal decomposer ────────────────────────────────────────────────────
    # goal_decomposer = Node(
    #     package='bringup',
    #     executable='goal_decomposer',
    #     name='goal_decomposer',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'path_sample_dist': 2.0,
    #         'gate_dist': 1.0,
    #         'min_goal_dist': 0.5,
    #         'plan_retry_delay_sec': 4.0,
    #         'nav2_settle_sec': 1.5,
    #     }]
    # )

#     carrot_goal = Node(
#     package='perception',
#     executable='carrot_goal',
#     name='carrot_goal_node',
#     output='screen',
#     parameters=[{
#         'use_sim_time': True,
#         'carrot_dist': 2.5,
#         'goal_tolerance': 0.8,
#     }]
# )

    lane_bev_carrot_node = Node(
        package='perception',
        executable='lane_bev_carrot',
        name='lane_bev_carrot',
        output='screen',
        parameters=[{
            'use_sim_time':          True,
            'carrot_dist_m':         4.8,
            'goal_tolerance':        0.8,
            'publish_rate':          2.0,
            'camera_hfov':           1.047,
            'image_width':           640,
            'image_height':          480,
            'min_proj_m':            0.3,
            'max_proj_m':            6.0,
            'n_bev_samples':         50,
            'fit_cache_sec':         1.0,
            'no_carrot_stop_streak': 3,
            'safe_cost_max': 50,
            'safety_radius':         0.6,  # NEW — footprint half-width + margin
            'max_carrot_dist_m': 6.0,
        }]
    )

   

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('bringup'),
            'config',
            'bringup.rviz'
        ])],
        output='screen'
    )

    return LaunchDescription([
        declare_xacro_file_arg,
        description,
        localization,
        planning,
        perception,
        # goal_decomposer,
        # carrot_goal,
        lane_bev_carrot_node,   
        rviz_node,
    ])