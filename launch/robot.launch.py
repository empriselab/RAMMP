import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    base_path_arg = DeclareLaunchArgument(
        'base_path',
        default_value=[EnvironmentVariable('HOME'), '/rammp_ws/src/RAMMP']
    )

    # Robot description via xacro
    robot_description = Command([
        'xacro ',
        os.path.join(
            get_package_share_directory('kortex_description'),
            'robots', 'gen3_robotiq_2f_85.xacro'
        ),
        ' dof:=7 vision:=false tool:=all'
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rob_st_pub',
        parameters=[{
            'robot_description': robot_description,
            'ignore_timestamp': True,
        }],
    )

    # Static transform: end_effector_link -> camera_link
    camera_link_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_broadcaster',
        arguments=['--x', '0.01', '--y', '0.0615', '--z', '0.03',
                   '--qx', '0.5', '--qy', '0.5', '--qz', '0.5', '--qw', '-0.5',
                   '--frame-id', 'end_effector_link', '--child-frame-id', 'camera_link'],
    )

    # Static transform: finger_tip -> drinkbase
    static_tf_drinking_tool = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_drinking_tool',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'finger_tip', '--child-frame-id', 'drinkbase'],
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_gui': False,
            'source_list': ['robot_joint_states', 'wrist_joint_states'],
            'rate': 100,
        }],
    )

    # Static transform: map -> world
    static_map2world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='st_map2world',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'map', '--child-frame-id', 'world'],
    )

    # Camera (RealSense)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py'
            )
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '1',
            'pointcloud.enable': 'true',
        }.items(),
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', [LaunchConfiguration('base_path'), '/config/real.rviz']],
    )

    # Simulated Robot (namespaced)
    sim_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rob_st_pub',
        namespace='sim',
        parameters=[{
            'robot_description': robot_description,
            'ignore_timestamp': True,
            'frame_prefix': 'sim/',
        }],
    )

    sim_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='sim',
        parameters=[{
            'use_gui': False,
            'source_list': ['robot_joint_states', 'wrist_joint_states'],
            'rate': 100,
        }],
    )

    sim_static_map2world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='st_map2world',
        namespace='sim',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'map', '--child-frame-id', 'sim/world'],
    )

    return LaunchDescription([
        base_path_arg,
        robot_state_publisher,
        camera_link_broadcaster,
        static_tf_drinking_tool,
        joint_state_publisher,
        static_map2world,
        realsense_launch,
        rviz_node,
        sim_robot_state_publisher,
        sim_joint_state_publisher,
        sim_static_map2world,
    ])
