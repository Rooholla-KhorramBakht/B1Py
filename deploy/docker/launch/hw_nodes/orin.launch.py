from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # launch the pointcloud to laser scan converter
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rslidar_to_laserscan.launch.py'])
        ),    

        # Launch the front looking D455 camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/d455.launch.py'])
        ),
        # Run the B1py node
        Node(
            package='b1py_node',
            executable='highlevel',
            name='b1_highlevel_node'
        ),
        # Launch the LiDAR sensor
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rslidar.launch.py'])
        ),
    ])
