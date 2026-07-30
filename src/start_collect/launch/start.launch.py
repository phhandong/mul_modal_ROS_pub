"""Launch the multimodal ROS 2 collection stack."""

from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def ars408_bag_recorder(context):
    """Start the guarded ARS408 recorder when radar recording is enabled."""
    radar_enabled = LaunchConfiguration('enable_ars408').perform(context).lower()
    record_enabled = LaunchConfiguration(
        'ars408_record_bag'
    ).perform(context).lower()
    true_values = ('1', 'true', 'yes', 'on')
    if radar_enabled not in true_values or record_enabled not in true_values:
        return []

    configured = LaunchConfiguration(
        'ars408_bag_output'
    ).perform(context).strip()
    output = (
        Path(configured).expanduser().resolve()
        if configured
        else Path.cwd()
        / 'bags'
        / f"ars408_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    return [
        ExecuteProcess(
            cmd=[
                'ros2',
                'run',
                'radar',
                'ars408_bag_recorder',
                str(output),
            ],
            name='ars408_bag_recorder',
            output='screen',
        )
    ]


def generate_launch_description():
    """Build the multimodal collection launch description."""
    enable_defaults = [
        ('enable_lidar', 'true'),
        ('enable_mono', 'true'),
        ('enable_bispectral', 'true'),
        ('enable_sync', 'true'),
        ('enable_radar', 'false'),
        ('enable_ars408', 'false'),
        ('enable_imu', 'false'),
        ('enable_nmea', 'false'),
        ('enable_hikvision_ros', 'false'),
        ('enable_rviz', 'false'),
    ]
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in enable_defaults
    ]
    arguments.extend(
        [
            DeclareLaunchArgument(
                'rslidar_config',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('start_collect'),
                        'config',
                        'rslidar_rs128.yaml',
                    ]
                ),
            ),
            DeclareLaunchArgument(
                'mono_config',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('start_collect'),
                        'config',
                        'hik_mono.yaml',
                    ]
                ),
            ),
            DeclareLaunchArgument(
                'bispectral_config',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('start_collect'),
                        'config',
                        'hik_bispectral.yaml',
                    ]
                ),
            ),
            DeclareLaunchArgument('ars408_source', default_value='gateway'),
            DeclareLaunchArgument(
                'ars408_gateway_ip', default_value='192.168.2.119'
            ),
            DeclareLaunchArgument(
                'ars408_gateway_port', default_value='29536'
            ),
            DeclareLaunchArgument(
                'ars408_gateway_transport',
                default_value='usr_canet_tcp_server',
            ),
            DeclareLaunchArgument(
                'ars408_can_interface', default_value='can0'
            ),
            DeclareLaunchArgument('ars408_sensor_id', default_value='0'),
            DeclareLaunchArgument(
                'ars408_frame_id', default_value='ars408'
            ),
            DeclareLaunchArgument(
                'ars408_min_speed_mps', default_value='0.0'
            ),
            DeclareLaunchArgument(
                'ars408_max_speed_mps', default_value='-1.0'
            ),
            DeclareLaunchArgument(
                'ars408_speed_filter_mode', default_value='magnitude'
            ),
            DeclareLaunchArgument(
                'ars408_parent_frame', default_value='ars408_world'
            ),
            DeclareLaunchArgument('ars408_x', default_value='0.0'),
            DeclareLaunchArgument('ars408_y', default_value='0.0'),
            DeclareLaunchArgument('ars408_z', default_value='0.0'),
            DeclareLaunchArgument('ars408_roll', default_value='0.0'),
            DeclareLaunchArgument('ars408_pitch', default_value='0.0'),
            DeclareLaunchArgument('ars408_yaw', default_value='0.0'),
            DeclareLaunchArgument(
                'ars408_record_bag', default_value='false'
            ),
            DeclareLaunchArgument('ars408_bag_output', default_value=''),
        ]
    )

    lidar = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        parameters=[
            {'config_path': LaunchConfiguration('rslidar_config')}
        ],
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        output='screen',
    )
    mono = Node(
        package='hik_tem',
        executable='hik_camera_node',
        name='hik_mono',
        parameters=[
            LaunchConfiguration('mono_config'),
            {'profile': 'mono'},
        ],
        condition=IfCondition(LaunchConfiguration('enable_mono')),
        output='screen',
    )
    bispectral = Node(
        package='hik_tem',
        executable='hik_camera_node',
        name='hik_bispectral',
        parameters=[
            LaunchConfiguration('bispectral_config'),
            {'profile': 'bispectral'},
        ],
        condition=IfCondition(LaunchConfiguration('enable_bispectral')),
        output='screen',
    )
    sync = Node(
        package='mess_sync',
        executable='mess_sync',
        condition=IfCondition(LaunchConfiguration('enable_sync')),
        output='screen',
    )
    quantum_radar = Node(
        package='radar',
        executable='quantum_radar',
        condition=IfCondition(LaunchConfiguration('enable_radar')),
        output='screen',
    )
    ars408 = Node(
        package='radar',
        executable='ars408_radar',
        name='ars408_radar',
        condition=IfCondition(LaunchConfiguration('enable_ars408')),
        parameters=[
            {
                'source': LaunchConfiguration('ars408_source'),
                'gateway_ip': LaunchConfiguration('ars408_gateway_ip'),
                'gateway_port': ParameterValue(
                    LaunchConfiguration('ars408_gateway_port'),
                    value_type=int,
                ),
                'gateway_transport': LaunchConfiguration(
                    'ars408_gateway_transport'
                ),
                'can_interface': LaunchConfiguration(
                    'ars408_can_interface'
                ),
                'sensor_id': ParameterValue(
                    LaunchConfiguration('ars408_sensor_id'),
                    value_type=int,
                ),
                'frame_id': LaunchConfiguration('ars408_frame_id'),
                'min_speed_mps': ParameterValue(
                    LaunchConfiguration('ars408_min_speed_mps'),
                    value_type=float,
                ),
                'max_speed_mps': ParameterValue(
                    LaunchConfiguration('ars408_max_speed_mps'),
                    value_type=float,
                ),
                'speed_filter_mode': LaunchConfiguration(
                    'ars408_speed_filter_mode'
                ),
            }
        ],
        output='screen',
    )
    ars408_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        condition=IfCondition(LaunchConfiguration('enable_ars408')),
        arguments=[
            '--x',
            LaunchConfiguration('ars408_x'),
            '--y',
            LaunchConfiguration('ars408_y'),
            '--z',
            LaunchConfiguration('ars408_z'),
            '--roll',
            LaunchConfiguration('ars408_roll'),
            '--pitch',
            LaunchConfiguration('ars408_pitch'),
            '--yaw',
            LaunchConfiguration('ars408_yaw'),
            '--frame-id',
            LaunchConfiguration('ars408_parent_frame'),
            '--child-frame-id',
            LaunchConfiguration('ars408_frame_id'),
        ],
        output='screen',
    )
    imu = Node(
        package='imu',
        executable='imu',
        condition=IfCondition(LaunchConfiguration('enable_imu')),
        output='screen',
    )
    nmea = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('opencpn2ros'),
                    'launch',
                    'nmea_parser.launch.py',
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration('enable_nmea')),
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution(
                [
                    FindPackageShare('start_collect'),
                    'rviz',
                    'rviz2.rviz',
                ]
            ),
        ],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )
    actions = [
        lidar,
        mono,
        bispectral,
        sync,
        quantum_radar,
        ars408,
        ars408_tf,
        imu,
        nmea,
        rviz,
        OpaqueFunction(function=ars408_bag_recorder),
    ]
    return LaunchDescription(arguments + actions)
