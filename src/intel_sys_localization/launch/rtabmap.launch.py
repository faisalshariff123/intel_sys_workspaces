import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('intel_sys_localization')
    default_rtabmap_params = PathJoinSubstitution([pkg_share, 'config', 'rtabmap.yaml'])

    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock')
    use_rgb_arg = DeclareLaunchArgument(
        'use_rgb', default_value='true',
        description='Fuse the RealSense RGB-D camera into the rtabmap SLAM node for loop closure / dense mapping. '
                    'Odometry comes from Point-LIO (/odom), not the camera. false = LiDAR-only mapping.'
    )
    odometry_arg = DeclareLaunchArgument(
        'odometry', default_value='point_lio', choices=['point_lio', 'icp'],
        description="Odometry source rtabmap consumes. 'point_lio' (default): Point-LIO's IMU-fused /odom — "
                    "stays level, the correct choice. 'icp': rtabmap's own LiDAR-only icp_odometry fallback — "
                    "no IMU, drifts in pitch on this rig; only for quick tests or when Point-LIO is unavailable."
    )
    launch_point_lio_arg = DeclareLaunchArgument(
        'launch_point_lio', default_value='true',
        description='Only applies when odometry:=point_lio. Also bring up the Point-LIO stack '
                    '(localization.launch.py) that rtabmap consumes via /odom. Set false when rtabmap is launched '
                    'alongside robot.launch.py, which already runs it.'
    )
    lidar_type_arg = DeclareLaunchArgument(
        'lidar_type', default_value='5', description='Point-LIO LiDAR Type (2: VELO16, 5: UNILIDAR). Forwarded to localization.launch.py.'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/lidar/imu',
        description="IMU source: '/lidar/imu' (Unitree L2 internal, 250Hz, matches Point-LIO's default) or "
                    "'/imu_raw' (chassis IMU from the STM32 bridge, frame imu_link)."
    )
    three_d_map_arg = DeclareLaunchArgument(
        'three_d_map', default_value='true',
        description='true = 3D voxel map + /octomap_* topics. false = ground-projected 2D occupancy grid only (/map, /grid_prob_map).'
    )
    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='/tmp/intel_sys_rtabmap.db',
        description='RTAB-Map session database path.'
    )
    delete_db_arg = DeclareLaunchArgument(
        'delete_db_on_start', default_value='true',
        description='Start a fresh map each launch instead of resuming the saved database (set false to keep building the same map across runs).'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rgb = LaunchConfiguration('use_rgb')
    imu_topic = LaunchConfiguration('imu_topic')
    three_d_map = LaunchConfiguration('three_d_map')
    database_path = LaunchConfiguration('database_path')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    launch_point_lio = LaunchConfiguration('launch_point_lio')
    lidar_type = LaunchConfiguration('lidar_type')
    odometry = LaunchConfiguration('odometry')

    # Flat PythonExpressions (do NOT nest one PythonExpression inside another —
    # the inner one does not evaluate, and the condition silently stays true).
    use_point_lio = PythonExpression(["'", odometry, "' == 'point_lio' and '", launch_point_lio, "' == 'true'"])
    use_icp = PythonExpression(["'", odometry, "' == 'icp'"])

    # 0. Optionally wipe the previous session's database so each launch starts a
    #    fresh map. Done ourselves with `rm -f` rather than relying on an
    #    rtabmap-internal delete flag — the installed binary's --help didn't
    #    show one, and this is correct regardless of version.
    delete_db_process = ExecuteProcess(
        cmd=['rm', '-f', database_path],
        shell=False,
        condition=IfCondition(delete_db_on_start)
    )

    # 1a. Odometry (default) — Point-LIO (tightly-coupled LiDAR-inertial), via the
    #    repo's localization.launch.py. Point-LIO fuses the IMU continuously, so
    #    gravity keeps roll/pitch level and the map doesn't tilt. rtabmap consumes
    #    its /odom (+ odom->base_footprint TF) below. Runs when odometry:=point_lio
    #    (default); skipped (so we don't double-launch it) when launch_point_lio
    #    :=false, e.g. alongside robot.launch.py which already brings it up.
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, 'launch', 'localization.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'lidar_type': lidar_type,
        }.items(),
        condition=IfCondition(use_point_lio),
    )

    # 1b. Odometry (fallback) — rtabmap's own icp_odometry: LiDAR-only ICP, no
    #    continuous IMU. It drifts in pitch on this rig (narrow Unitree L2 vertical
    #    FOV) and tilts the map, so it is NOT the default — kept only as an opt-in
    #    (odometry:=icp) for quick tests or when Point-LIO isn't available. Publishes
    #    the same /odom + odom->base_footprint TF that rtabmap consumes below, so
    #    nothing else changes when switching.
    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[
            default_rtabmap_params,
            {
                'use_sim_time': use_sim_time,
                'wait_imu_to_init': True,
            }
        ],
        remappings=[
            ('scan_cloud', '/lidar/points'),
            ('imu', imu_topic),
        ],
        condition=IfCondition(use_icp),
    )

    # 2. RTAB-Map SLAM node — loop closure, graph optimization, 2D/3D map output.
    #    This is the node that actually fuses RGB-D + LiDAR + IMU together
    #    (subscribe_rgb/subscribe_depth/subscribe_scan_cloud/imu all verified
    #    as real subscriptions on the installed rtabmap_slam binary).
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            default_rtabmap_params,
            {
                'use_sim_time': use_sim_time,
                'subscribe_rgb': use_rgb,
                'subscribe_depth': use_rgb,
                'database_path': database_path,
                # rtabmap.yaml declares every RTAB-Map param (including Grid/3D) as a
                # string, matching the library's internal string-keyed param system —
                # ROS2 would otherwise auto-infer this LaunchConfiguration as bool and
                # crash with InvalidParameterTypeException on a string/bool mismatch
                # (hit this for real: rtabmap_node died on startup before this fix).
                'Grid/3D': ParameterValue(three_d_map, value_type=str),
            }
        ],
        remappings=[
            ('scan_cloud', '/lidar/points'),
            ('imu', imu_topic),
            ('odom', '/odom'),
            ('rgb/image', '/camera/image_raw'),
            ('depth/image', '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rgb_arg,
        imu_topic_arg,
        three_d_map_arg,
        database_path_arg,
        delete_db_arg,
        odometry_arg,
        launch_point_lio_arg,
        lidar_type_arg,
        delete_db_process,
        localization_launch,
        icp_odometry_node,
        rtabmap_node,
    ])
