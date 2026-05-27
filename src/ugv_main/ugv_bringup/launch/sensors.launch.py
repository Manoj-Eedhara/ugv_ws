from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='ugv',
        description='Namespace prefix for all robot topics  (e.g. ugv → /ugv/scan)'
    )
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')

    ns = LaunchConfiguration('robot_name')

    def nst(path):
        return PythonExpression(["'/' + '", ns, "' + '/" + path + "'"])

    pkg_desc    = get_package_share_directory('ugv_description')

    # Robot model — provides base_footprint → base_lidar_link and base_imu_link TF
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'bringup',
        }.items()
    )

    # MCU serial comms — raw IMU from the base controller
    # Publishes: /{ns}/imu/data_raw, /{ns}/imu/mag, /{ns}/voltage
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

    # IMU complementary filter — adds orientation quaternion
    # Subscribes: /{ns}/imu/data_raw   Publishes: /{ns}/imu/data
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

    # Motor / servo / LED driver
    # Subscribes: /{ns}/cmd_vel, /{ns}/joint_states, /{ns}/led_ctrl
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

    # LiDAR LD06
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

    return LaunchDescription([
        robot_name_arg,
        use_rviz_arg,
        robot_state_launch,
        bringup_node,
        driver_node,
        imu_filter_node,
        ldlidar_node,
    ])
