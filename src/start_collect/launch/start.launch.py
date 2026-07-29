from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    args=[DeclareLaunchArgument(name, default_value=value) for name,value in [('enable_lidar','true'),('enable_mono','true'),('enable_bispectral','true'),('enable_sync','true'),('enable_radar','false'),('enable_imu','false'),('enable_nmea','false'),('enable_hikvision_ros','false'),('enable_rviz','false')]]
    args.append(DeclareLaunchArgument('rslidar_config', default_value=PathJoinSubstitution([FindPackageShare('start_collect'),'config','rslidar_rs128.yaml'])))
    args.append(DeclareLaunchArgument('mono_config', default_value=PathJoinSubstitution([FindPackageShare('start_collect'),'config','hik_mono.yaml'])))
    args.append(DeclareLaunchArgument('bispectral_config', default_value=PathJoinSubstitution([FindPackageShare('start_collect'),'config','hik_bispectral.yaml'])))
    lidar=Node(package='rslidar_sdk', executable='rslidar_sdk_node', name='rslidar_sdk_node', parameters=[{'config_path':LaunchConfiguration('rslidar_config')}], condition=IfCondition(LaunchConfiguration('enable_lidar')), output='screen')
    mono=Node(package='hik_tem', executable='hik_camera_node', name='hik_mono', parameters=[LaunchConfiguration('mono_config'), {'profile':'mono'}], condition=IfCondition(LaunchConfiguration('enable_mono')), output='screen')
    bi=Node(package='hik_tem', executable='hik_camera_node', name='hik_bispectral', parameters=[LaunchConfiguration('bispectral_config'), {'profile':'bispectral'}], condition=IfCondition(LaunchConfiguration('enable_bispectral')), output='screen')
    sync=Node(package='mess_sync', executable='mess_sync', condition=IfCondition(LaunchConfiguration('enable_sync')), output='screen')
    radar=Node(package='radar', executable='quantum_radar', condition=IfCondition(LaunchConfiguration('enable_radar')), output='screen')
    imu=Node(package='imu', executable='imu', condition=IfCondition(LaunchConfiguration('enable_imu')), output='screen')
    nmea=IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('opencpn2ros'),'launch','nmea_parser.launch.py'])), condition=IfCondition(LaunchConfiguration('enable_nmea')))
    rviz=Node(package='rviz2', executable='rviz2', arguments=['-d',PathJoinSubstitution([FindPackageShare('start_collect'),'rviz','rviz2.rviz'])], condition=IfCondition(LaunchConfiguration('enable_rviz')))
    return LaunchDescription(args+[lidar,mono,bi,sync,radar,imu,nmea,rviz])
