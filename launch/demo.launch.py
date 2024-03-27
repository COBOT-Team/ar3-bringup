from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch, generate_move_group_launch, generate_rsp_launch, generate_spawn_controllers_launch
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():

    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder(
            "cobot0", package_name="single_cobot_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory('single_cobot_moveit_config'),
                'config',
                'cobot0.urdf.xacro'
            )
        )
    ).to_moveit_configs()

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="demo.rviz",
        description="RViz configuration file"
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ar3_bringup"), "config", rviz_base])

    spawn_controllers = generate_spawn_controllers_launch(moveit_config)
    move_group = generate_move_group_launch(moveit_config)
    static_tf = generate_static_virtual_joint_tfs_launch(moveit_config)
    robot_state = generate_rsp_launch(moveit_config)

    static_tf_table = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="table_transforms_pub",
        output="log",
        arguments=['--x', "0",
                   '--y', "0",
                   '--z', "0",
                   '--yaw', '0',
                   '--pitch', '0',
                   '--roll', '0',
                   '--frame-id', "world",
                   '--child-frame-id', "table_frame"
                   ]
    )

    ros2_controllers_path = os.path.join(get_package_share_directory(
        "single_cobot_moveit_config"), "config", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description")
        ],
        output="both",
    )

    kinect2ros = Node(
        package='kinect2ros',
        executable='kinect2ros',
        name='kinect2ros'
    )

    aruco_transforms = Node(
        package='aruco_transforms',
        executable='aruco_transforms',
        name='aruco_transforms'
    )

    tof_piece_finder = Node(
        package='tof_piece_finder',
        executable='tof_piece_finder',
        name='tof_piece_finder'
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    chess_controller = Node(
        package='chess_controller',
        executable='chess_controller',
        name="chess_controller"
    )

    chess_player = Node(
        package='chess_player',
        executable='chess_player',
        name="chess_player",
        parameters=[
            {'cobot_ns': 'cobot0'}
        ]
    )

    return LaunchDescription([
        rviz_config_arg,

        static_tf_table,
        ros2_control_node,
        spawn_controllers,
        move_group,
        robot_state,
        static_tf,
        kinect2ros,
        aruco_transforms,
        tof_piece_finder,
        rviz,
        chess_controller,
        chess_player,
    ])
