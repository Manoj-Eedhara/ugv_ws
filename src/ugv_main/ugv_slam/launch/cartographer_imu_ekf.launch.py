from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='ugv',
        description='Namespace prefix for all robot topics  (e.g. ugv → /ugv/scan)'
    )

    ns = LaunchConfiguration('robot_name')

    def nst(path):
        """Substitution that evaluates to '/{robot_name}/{path}', e.g. '/ugv/scan'."""
        return PythonExpression(["'/' + '", ns, "' + '/" + path + "'"])

    pkg_carto   = get_package_share_directory('cartographer')
    pkg_bringup = get_package_share_directory('ugv_bringup')
    pkg_desc    = get_package_share_directory('ugv_description')
    pkg_rpp     = get_package_share_directory('robot_pose_publisher')

    # --- Robot model (TF, URDF — always global) ---
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'slam_2d',
        }.items()
    )

    # --- Serial comms / raw sensor data ---
    # Publishes: /{ns}/imu/data_raw, /{ns}/imu/mag, /{ns}/odom/odom_raw, /{ns}/voltage
    # Subscribes: /{ns}/serial_cmd
    bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
        remappings=[
            ('imu/data_raw',   nst('imu/data_raw')),
            ('imu/mag',        nst('imu/mag')),
            ('odom/odom_raw',  nst('odom/odom_raw')),
            ('voltage',        nst('voltage')),
            ('ugv/serial_cmd', nst('serial_cmd')),
        ]
    )

    # --- Motor / servo / LED driver ---
    # Subscribes: /{ns}/cmd_vel, /{ns}/joint_states, /{ns}/led_ctrl, /{ns}/voltage
    # Publishes:  /{ns}/serial_cmd
    driver_node = Node(
        package='ugv_bringup',
        executable='ugv_driver',
        remappings=[
            ('cmd_vel',          nst('cmd_vel')),
            ('voltage',          nst('voltage')),
            ('ugv/serial_cmd',   nst('serial_cmd')),
            ('ugv/joint_states', nst('joint_states')),
            ('ugv/led_ctrl',     nst('led_ctrl')),
        ]
    )

    # --- LiDAR LD06 (inlined from ld06.launch.py to control topic_name param) ---
    # Publishes: /{ns}/scan
    ldlidar_node = Node(
        package='ldlidar',
        executable='ldlidar_node',
        name='LD06',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD06'},
            {'topic_name': nst('scan')},
            {'frame_id': 'base_lidar_link'},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': True},
            {'angle_crop_min': 225.0},
            {'angle_crop_max': 315.0},
        ]
    )

    # --- IMU complementary filter: /{ns}/imu/data_raw → /{ns}/imu/data ---
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
        remappings=[
            ('imu/data_raw', nst('imu/data_raw')),
            ('imu/mag',      nst('imu/mag')),
            ('imu/data',     nst('imu/data')),
        ]
    )

    # --- Wheel encoder + IMU → /{ns}/odom_raw (nav_msgs/Odometry) ---
    base_node_ekf = Node(
        package='ugv_base_node',
        executable='base_node_ekf',
        parameters=[{'pub_odom_tf': False}],
        remappings=[
            ('imu/data',      nst('imu/data')),
            ('odom/odom_raw', nst('odom/odom_raw')),
            ('odom_raw',      nst('odom_raw')),
        ]
    )

    # --- Laser scan odometry: /{ns}/scan → /{ns}/odom_rf2o, no TF ---
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': nst('scan'),
            'odom_topic':       nst('odom_rf2o'),
            'imu_topic':        nst('imu/data'),
            'publish_tf':       False,
            'base_frame_id':    'base_footprint',
            'odom_frame_id':    'odom',
            'freq':             20.0,
        }]
    )

    # --- EKF: fuse /{ns}/odom_raw + /{ns}/odom_rf2o + /{ns}/imu/data → /{ns}/odom ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'param', 'ekf_imu_laser.yaml'),
            {   # Override source topic names from yaml with namespaced versions
                'odom0': nst('odom_raw'),
                'odom1': nst('odom_rf2o'),
                'imu0':  nst('imu/data'),
            }
        ],
        remappings=[('/odometry/filtered', nst('odom'))]
    )

    # --- Robot pose publisher (geometry_msgs/Pose on /robot_pose) ---
    robot_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rpp, 'launch', 'robot_pose_publisher_launch.py')
        )
    )

    # --- Cartographer SLAM (map/TF are always global — only scan is namespaced) ---
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_carto, 'config'),
            '-configuration_basename', 'mapping_2d_imu_ekf.lua',
        ],
        remappings=[
            ('scan', nst('scan')),
        ]
    )

    cartographer_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    return LaunchDescription([
        use_rviz_arg,
        robot_name_arg,
        robot_state_launch,
        bringup_node,
        driver_node,
        ldlidar_node,
        imu_filter_node,
        base_node_ekf,
        rf2o_node,
        ekf_node,
        robot_pose_launch,
        cartographer_node,
        cartographer_grid_node,
    ])
