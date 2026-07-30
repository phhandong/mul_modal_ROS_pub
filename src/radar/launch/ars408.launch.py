from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def bag_recorder(context):
    enabled = LaunchConfiguration("record_bag").perform(context).lower()
    if enabled not in ("1", "true", "yes", "on"):
        return []
    configured = LaunchConfiguration("bag_output").perform(context).strip()
    if configured:
        output = Path(configured).expanduser().resolve()
    else:
        output = (
            Path.cwd()
            / "bags"
            / f"ars408_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    return [ExecuteProcess(
        cmd=[
            "ros2", "run", "radar", "ars408_bag_recorder", str(output),
        ],
        name="ars408_bag_recorder",
        output="screen",
    )]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("source", default_value="gateway"),
        DeclareLaunchArgument("gateway_ip", default_value="192.168.2.119"),
        DeclareLaunchArgument("gateway_port", default_value="29536"),
        DeclareLaunchArgument("gateway_transport", default_value="usr_canet_tcp_server"),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("sensor_id", default_value="0"),
        DeclareLaunchArgument("frame_id", default_value="ars408"),
        DeclareLaunchArgument("min_speed_mps", default_value="0.0"),
        DeclareLaunchArgument("max_speed_mps", default_value="-1.0"),
        DeclareLaunchArgument("speed_filter_mode", default_value="magnitude"),
        DeclareLaunchArgument("enable_rviz", default_value="false"),
        DeclareLaunchArgument("publish_static_tf", default_value="true"),
        DeclareLaunchArgument("parent_frame", default_value="ars408_world"),
        DeclareLaunchArgument("radar_x", default_value="0.0"),
        DeclareLaunchArgument("radar_y", default_value="0.0"),
        DeclareLaunchArgument("radar_z", default_value="0.0"),
        DeclareLaunchArgument("radar_roll", default_value="0.0"),
        DeclareLaunchArgument("radar_pitch", default_value="0.0"),
        DeclareLaunchArgument("radar_yaw", default_value="0.0"),
        DeclareLaunchArgument("record_bag", default_value="false"),
        DeclareLaunchArgument("bag_output", default_value=""),
    ]
    parameters = {
        "source": LaunchConfiguration("source"),
        "gateway_ip": LaunchConfiguration("gateway_ip"),
        "gateway_port": ParameterValue(LaunchConfiguration("gateway_port"), value_type=int),
        "gateway_transport": LaunchConfiguration("gateway_transport"),
        "can_interface": LaunchConfiguration("can_interface"),
        "sensor_id": ParameterValue(LaunchConfiguration("sensor_id"), value_type=int),
        "frame_id": LaunchConfiguration("frame_id"),
        "min_speed_mps": ParameterValue(LaunchConfiguration("min_speed_mps"), value_type=float),
        "max_speed_mps": ParameterValue(LaunchConfiguration("max_speed_mps"), value_type=float),
        "speed_filter_mode": LaunchConfiguration("speed_filter_mode"),
    }
    radar = Node(
        package="radar", executable="ars408_radar", name="ars408_radar",
        parameters=[parameters], output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("radar"), "rviz", "ars408.rviz"
        ])],
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        output="screen",
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", LaunchConfiguration("radar_x"),
            "--y", LaunchConfiguration("radar_y"),
            "--z", LaunchConfiguration("radar_z"),
            "--roll", LaunchConfiguration("radar_roll"),
            "--pitch", LaunchConfiguration("radar_pitch"),
            "--yaw", LaunchConfiguration("radar_yaw"),
            "--frame-id", LaunchConfiguration("parent_frame"),
            "--child-frame-id", LaunchConfiguration("frame_id"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_static_tf")),
        output="screen",
    )
    return LaunchDescription(
        arguments + [radar, static_tf, rviz, OpaqueFunction(function=bag_recorder)]
    )
