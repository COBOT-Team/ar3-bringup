from launch import LaunchDescription
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python import get_package_share_directory
import os
def generate_launch_description():

    # Arguments
    serial_port = LaunchConfiguration("port")
    serial_port_launch_arg = DeclareLaunchArgument(
        "port", default_value=TextSubstitution(text="/dev/ttyCobot0")
    )
    baudrate = LaunchConfiguration("baudrate")
    baudrate_launch_arg = DeclareLaunchArgument(
        "baudrate", default_value=TextSubstitution(text="115200")
    )
    namespace = LaunchConfiguration("namespace")
    namespace_launch_arg = DeclareLaunchArgument(
        "namespace", default_value=TextSubstitution(text="cobot0")
    )

    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("ar3", package_name="ar3_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory('ar3_moveit_config'),
                'config',
                'ar3.urdf.xacro'
            ),
            mappings={
                "serial_port": serial_port,
                "baud_rate": baudrate
            }
        )
    ).to_moveit_configs()

    # Create demo launch description
    demo_desc = generate_demo_launch(moveit_config)

    # Prepend namespace to launch description
    demo_desc_with_ns = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            demo_desc
        ]
    )

    return LaunchDescription([
        serial_port_launch_arg,
        baudrate_launch_arg,
        namespace_launch_arg,

        demo_desc  # Namespaced version does not work
    ])
