from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('bag_path'), ExecuteProcess(cmd=['ros2','bag','play',LaunchConfiguration('bag_path'),'--clock']), Node(package='start_collect', executable='start_collect.py', output='screen')])
