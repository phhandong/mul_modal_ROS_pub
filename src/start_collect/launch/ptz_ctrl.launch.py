from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description(): return LaunchDescription([Node(package='hik_tem', executable='ptz_ctrl.py', output='screen')])
