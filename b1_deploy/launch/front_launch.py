from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction

def generate_launch_description():
    # Replace 'your_package_name' with the name of your package
    pkg_dir = get_package_share_directory('realsense2_camera')
    launch1_path = os.path.join(pkg_dir, 'launch', 'front_front_launch.py')
    launch2_path = os.path.join(pkg_dir, 'launch', 'front_down_launch.py')

    # Include launch files
    launch1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch1_path])
    )
    launch2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch2_path])
    )

    # Launch description
    return LaunchDescription([
        launch1,
        TimerAction(
            period=5,
            actions=[launch2]
        )]

