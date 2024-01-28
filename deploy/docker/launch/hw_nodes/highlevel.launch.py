from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([   
        # Run the B1py node
        Node(
            package='b1py_node',
            executable='highlevel',
            name='b1_highlevel_node'
        ),

        # Run the B1py calibration TF broadcaster
        Node(
            package='b1py_calib',
            executable='calib_broadcaster',
            name='b1_calib_broadcaster_node'
        ),
    ])
