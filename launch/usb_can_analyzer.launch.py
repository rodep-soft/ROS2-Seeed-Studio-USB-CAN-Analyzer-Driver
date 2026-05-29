from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "seeed_usb_can_analyzer_driver"

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare(package_name),
                        "config",
                        "usb_can_analyzer.yaml",
                    ]
                ),
                description="Path to the USB-CAN analyzer parameter YAML file.",
            ),
            Node(
                package=package_name,
                executable="usb_can_analyzer_node",
                name="usb_can_analyzer_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
