from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scene_config = LaunchConfiguration("scene_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "scene_config",
            default_value="wheelchair",
        ),
        DeclareLaunchArgument(
            "no_waits",
            default_value="false",
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="st_map2world",
            arguments=["0", "0", "0", "0", "0", "0", "1", "map", "world"],
        ),

        Node(
            package="drink_actions_test",
            executable="drink_action_server.py",
            name="drink_action_server",
            output="screen",
            arguments=["--scene_config", scene_config, "--run_on_robot", "--no_waits"],
        ),
    ])
