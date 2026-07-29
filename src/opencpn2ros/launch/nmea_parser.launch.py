from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package='opencpn2ros', executable='arpa_parser', parameters=[{'port': 20004}]),
        Node(package='opencpn2ros', executable='ais_parser', parameters=[{'port': 20002}]),
    ])
