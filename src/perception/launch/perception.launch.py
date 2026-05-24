from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )

    lane_detection_node = Node(
        package='perception',
        executable='lane_detection',
        name='lane_detection_node',
        output='screen',
        parameters=[{
            'use_sim_time':       True,
            'image_topic':        '/camera/image_raw',
            'show_debug':         True,

            'roi_top_frac':       0.4,

            'white_v_min':        160,
            'white_s_max':        60,

            'close_kw':           7,
            'close_kh':           30,

            'hough_threshold':    10,
            'hough_min_len':      15.0,
            'hough_max_gap':      50.0,

            'min_slope_abs':      0.2,
            'max_slope_abs':      4.0,
            'lane_half_width_px': 160.0,
            'min_lane_sep_px':    120.0,
            'ema_alpha':          0.30,
        }]
    )

    lane_costmap = Node(
        package='perception',
        executable='lane_costmap',
        name='lane_costmap',
        output='screen',
        parameters=[{
            'use_sim_time':   True,

            # Map extent — must match global_costmap.yaml
            'map_width_m':    70.0,
            'map_height_m':   70.0,
            'resolution':      0.10,
            'map_origin_x':  -35.0,
            'map_origin_y':  -35.0,

            # Camera
            'camera_hfov':   1.047,
            'image_width':   640,
            'image_height':  480,
            'roi_top_frac':  0.4,

            # Detection
            'white_v_min':   130,
            'white_s_max':   80,
            'sample_rows':   8,

            # ── Pothole filter (minAreaRect aspect only) ─────────────────
            # blob_min_aspect is the ONLY shape filter used.
            # Potholes → minAreaRect aspect ≈ 1.0–1.8
            # Lane stripes (any angle/distance) → aspect ≥ 2.5
            # Works on straight roads AND curves.
            'blob_min_aspect':  2.5,
            'blob_min_area':    60,

            # Hough
            'hough_min_len':    25.0,
            'hough_max_gap':    40.0,
            'hough_threshold':  15,

            # Loose span sanity check — keep very low so short dashes pass
            'min_span_frac':    0.03,

            # Projection
            'lethal_band_px':   20,
            'obstacle_pixels_outside': 48,
            'free_pixels_inside':      32,

            # Performance
            'publish_rate':     5.0,
            'process_every_n':  3,

            'blob_max_circularity' : 0.35,  
        }]
    )

    lane_assist = Node(
        package='perception',
        executable='lane_assist_node',
        name='lane_assist_node',
        output='screen',
        parameters=[{
            'use_sim_time':     True,
            'Kp':               0.18,
            'Kd':               0.08,
            'max_correction':   0.3,
            'image_half_width': 320.0,
            'dead_band_px':     25.0,
            'timeout_sec':      0.5,
        }]
    )

    pothole_costmap = Node(
        package='perception',
        executable='pothole_costmap',
        name='pothole_costmap',
        output='screen',
        parameters=[{
            'use_sim_time':          True,
            # Map extent — must match global_costmap.yaml
            'map_width_m':           70.0,
            'map_height_m':          70.0,
            'resolution':             0.10,
            'map_origin_x':         -35.0,
            'map_origin_y':         -35.0,
            # Camera
            'camera_hfov':           1.047,
            'image_width':           640,
            'image_height':          480,
            'roi_top_frac':          0.35,
            # Detection thresholds
            'white_v_min':           130,
            'white_s_max':            80,
            'blob_min_area':         400,
            'blob_min_circularity':  0.40,
            'blob_max_aspect':       2.2,
            # World-space pothole size
            'min_pothole_r':         0.20,
            'max_pothole_r':         1.20,
            'inflation_pad':         0.15,
            'radius_samples':        12,
            # Performance
            'publish_rate':          2.0,
            'process_every_n':       5,
            # Projection safety
            'max_proj_m':            4.5,
            'min_proj_m':            0.3,
            'forward_only':          True,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        lane_detection_node,
        lane_costmap,
        lane_assist,
        pothole_costmap,   # ← new
    ])