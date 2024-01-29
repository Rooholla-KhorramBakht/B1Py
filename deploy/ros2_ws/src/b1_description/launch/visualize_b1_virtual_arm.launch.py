import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    user_debug_parameter_name = "user_debug"
    user_debug = LaunchConfiguration(user_debug_parameter_name)

    b1_xacro_file = os.path.join(
        get_package_share_directory("b1_description"), "robots", "robot_virtual_arm.xacro"
    )
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", b1_xacro_file, " DEBUG:=", user_debug]
    )

    rviz_file = os.path.join(
        get_package_share_directory("b1_description"), "rviz", "visualize_b1.rviz"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                user_debug_parameter_name,
                default_value="false",
                description="debug or not",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[
                    ("/joint_states", "/b1/joint_states"),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
            ),
        ]
    )
