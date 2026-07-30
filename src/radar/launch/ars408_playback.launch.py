from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def bag_player(context):
    configured = LaunchConfiguration("bag_path").perform(context).strip()
    if not configured:
        raise RuntimeError("bag_path is required")
    path = Path(configured).expanduser().resolve()
    if not path.exists():
        raise RuntimeError(f"bag_path does not exist: {path}")
    playback_input_topic = LaunchConfiguration(
        "playback_input_topic"
    ).perform(context)
    command = [
        "ros2", "bag", "play", str(path),
        "--topics", "/ars408/points_raw",
        "--remap", f"/ars408/points_raw:={playback_input_topic}",
    ]
    if LaunchConfiguration("loop").perform(context).lower() in (
        "1", "true", "yes", "on"
    ):
        command.append("--loop")
    return [ExecuteProcess(cmd=command, name="ars408_bag_player", output="screen")]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("bag_path", default_value=""),
        DeclareLaunchArgument("loop", default_value="false"),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        DeclareLaunchArgument(
            "playback_input_topic",
            default_value="/ars408/playback/points_raw",
        ),
        DeclareLaunchArgument("min_speed_mps", default_value="0.0"),
        DeclareLaunchArgument("max_speed_mps", default_value="-1.0"),
        DeclareLaunchArgument("speed_filter_mode", default_value="magnitude"),
    ]
    filter_node = Node(
        package="radar",
        executable="ars408_speed_filter",
        name="ars408_speed_filter",
        parameters=[{
            "input_topic": LaunchConfiguration("playback_input_topic"),
            "min_speed_mps": ParameterValue(
                LaunchConfiguration("min_speed_mps"), value_type=float
            ),
            "max_speed_mps": ParameterValue(
                LaunchConfiguration("max_speed_mps"), value_type=float
            ),
            "speed_filter_mode": LaunchConfiguration("speed_filter_mode"),
        }],
        output="screen",
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "ars408_world",
            "--child-frame-id", "ars408",
        ],
        output="screen",
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
    return LaunchDescription(
        arguments + [
            filter_node,
            static_tf,
            rviz,
            OpaqueFunction(function=bag_player),
        ]
    )
