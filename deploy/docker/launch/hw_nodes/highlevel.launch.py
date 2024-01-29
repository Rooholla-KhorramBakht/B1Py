from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    b1_xacro_file = os.path.join(
        get_package_share_directory("b1_description"), "robots", "robot.xacro"
    )
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", b1_xacro_file, " DEBUG:=", 'false']
    )
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
        Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description,
                             "frame_prefix": 'b1/'}],
                remappings=[
                    ("/joint_states", f"/b1/joint_states"),
                    ("/robot_description", f"/b1/robot_description")
                ],
            ),
    ])
