from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scene_config = LaunchConfiguration("scene_config")
    rviz_config = LaunchConfiguration("rviz_config")

    # robot_description = ParameterValue(
    #     Command([
    #         "xacro ",
    #         PathJoinSubstitution([
    #             FindPackageShare("kortex_description"),
    #             "robots",
    #             "gen3_robotiq_2f_85.xacro",
    #         ]),
    #         " dof:=7 vision:=false",
    #     ]),
    #     value_type=str,
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            "scene_config",
            default_value="wheelchair",
        ),

        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("drink_actions_test"),
                "config",
                "real.rviz",
            ]),
        ),

        # Node(
        #     package="robot_state_publisher",
        #     executable="robot_state_publisher",
        #     name="rob_st_pub",
        #     parameters=[
        #         {"robot_description": robot_description},
        #         {"ignore_timestamp": True},
        #     ],
        #     output="screen",
        # ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[
                {"use_gui": False},
                {"source_list": ["robot_joint_states", "wrist_joint_states"]},
                {"rate": 100},
            ],
            output="screen",
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_link_broadcaster",
            arguments=[
                "0.01", "0.0615", "0.03",
                "0.5", "0.5", "0.5", "-0.5",
                "end_effector_link",
                "camera_link",
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_drinking_tool",
            arguments=[
                "0", "0", "0",
                "0", "0", "0", "1",
                "finger_tip",
                "drinkbase",
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="st_map2world",
            arguments=[
                "0", "0", "0",
                "0", "0", "0", "1",
                "map",
                "world",
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                ])
            ),
            launch_arguments={
                "align_depth.enable": "true",
                "enable_sync": "true",
                "enable_gyro": "true",
                "enable_accel": "true",
                "unite_imu_method": "linear_interpolation",
                "pointcloud.enable": "true",
            }.items(),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", rviz_config],
            output="screen",
        ),
        
        Node(
            package="drink_actions_test",
            executable="drink_action_server.py",
            name="drink_action_server",
            output="screen",
            arguments=["--scene_config", scene_config],
        ),

        GroupAction([
            PushRosNamespace("sim"),

            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[
                    {"use_gui": False},
                    {"source_list": ["robot_joint_states", "wrist_joint_states"]},
                    {"rate": 100},
                ],
            ),

            # Node(
            #     package="robot_state_publisher",
            #     executable="robot_state_publisher",
            #     name="rob_st_pub",
            #     parameters=[
            #         {"robot_description": robot_description},
            #         {"ignore_timestamp": True},
            #         {"frame_prefix": "sim/"},
            #     ],
            # ),

            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="st_map2world",
                arguments=[
                    "0", "0", "0",
                    "0", "0", "0", "1",
                    "map",
                    "sim/world",
                ],
            ),
        ]),
    ])