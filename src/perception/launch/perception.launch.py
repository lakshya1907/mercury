from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )
    use_sim = LaunchConfiguration('use_sim_time')

    lane_detection_node = Node(
        package='perception',
        executable='lane_detection',
        name='lane_detection_node',
        output='screen',
        parameters=[{
            'use_sim_time':          True,
            'image_topic':           '/camera/image_raw',
            'show_debug':            True,

            'roi_top_frac':          0.25,   # as set before

            # ── THESE ARE THE BROKEN ONES ──────────────────────────────
            # Sim at 1m+steeper pitch renders lane markings darker
            # Drop threshold significantly
            'white_v_min':           160,    # was 170 — too strict now
            'white_s_max':           60,     # was 60  — slightly more permissive

            # Also loosen the morphological close to catch thinner marks
            'close_kw':              7,      # was 5
            'close_kh':              30,     # was 25

            # Loosen Hough — fewer bright pixels means fewer edge points
            'hough_threshold':       10,     # was 15
            'hough_min_len':         15.0,   # was 20.0
            'hough_max_gap':         50.0,   # was 40.0
            # ────────────────────────────────────────────────────────────

            'min_slope_abs':         0.2,
            'max_slope_abs':         4.0,
            'lane_half_width_px':    160.0,
            'use_auto_cal':          False,
            'max_valid_sep_px':      450.0,
            'min_lane_sep_px':       120.0,
            'ema_alpha':             0.30,
            'drift_gain':            0.8,
        }]
    )

    lane_costmap = Node(
        package='perception',
        executable='lane_costmap',
        name='lane_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,

            # ── Map extent — must match global_costmap.yaml ──────────────
            'map_width_m':    70.0,
            'map_height_m':   70.0,
            'resolution':      0.10,   # 0.10 m/cell → 700×700 grid
            'map_origin_x':  -35.0,
            'map_origin_y':  -35.0,

            # ── Camera sensor (matches URDF robot_sensors.xacro) ─────────
            'camera_hfov':   1.047,    # rad  (from sensor config)
            'image_width':   640,
            'image_height':  480,

            # ── Detection tuning ─────────────────────────────────────────
            'roi_top_frac':  0.35,     # ignore top 35% (sky, far distance)
            'sample_rows':   8,        # rows to project per frame
            'white_v_min':   130,   # was 170
            'white_s_max':   80,    # was 60      # HSV saturation threshold

            # ── Projection extent ─────────────────────────────────────────
            # Pixels projected outside each boundary → marked lethal (100).
            # 48 px × ~0.005 m/px (close range) ≈ 0.25 m outside the line.
            'obstacle_pixels_outside': 48,
            # Pixels projected inside boundary → confirmed free (0).
            'free_pixels_inside':      32,

            # ── Performance ───────────────────────────────────────────────
            'publish_rate':    5.0,    # Hz  (StaticLayer re-reads each update)
            'process_every_n': 3,      # process 1-in-3 camera frames (~10 Hz)
        }]
    )

     # ── Lane assist node ───────────────────────────────────────────────────
    lane_assist = Node(
        package='perception',
        executable='lane_assist_node',
        name='lane_assist_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'Kp': 0.18,
            'Kd': 0.08,
            'max_correction': 0.3,
            'image_half_width': 320.0,
            'dead_band_px': 25.0,
            'timeout_sec': 0.5,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        lane_detection_node,
        lane_costmap,
        lane_assist,
    ])