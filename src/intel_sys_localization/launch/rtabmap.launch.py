import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
        description='Fuse the RealSense RGB-D camera into the rtabmap SLAM node (loop closure / mapping only — '
                    'odometry is always LiDAR-only, icp_odometry has no RGB-D input). false = LiDAR-only mapping.'
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

    # 0. Optionally wipe the previous session's database so each launch starts a
    #    fresh map. Done ourselves with `rm -f` rather than relying on an
    #    rtabmap-internal delete flag — the installed binary's --help didn't
    #    show one, and this is correct regardless of version.
    delete_db_process = ExecuteProcess(
        cmd=['rm', '-f', database_path],
        shell=False,
        condition=IfCondition(delete_db_on_start)
    )

    # 1. Odometry — ICP against the LiDAR cloud. LiDAR-only: icp_odometry has no
    #    RGB-D subscription (verified against the installed rtabmap_odom binary),
    #    so the camera does not assist pose tracking here, only mapping below.
    #    IMU fusion (wait_imu_to_init + 'imu' remap) is the documented upstream
    #    mechanism but wasn't observed to attach a subscriber in local testing —
    #    confirm with `ros2 topic info <imu_topic> -v` once this is running
    #    against real sensor data before relying on it.
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
        ]
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
        delete_db_process,
        icp_odometry_node,
        rtabmap_node,
    ])
