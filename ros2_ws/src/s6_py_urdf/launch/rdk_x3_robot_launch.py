import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the URDF file
    urdf_file = os.path.join(
        get_package_share_directory('s6_py_urdf'),
        'urdf',
        'rdk_x3_robot.urdf'
    )

    # Path to the RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory('s6_py_urdf'),
        'rviz',
        'rdk_x3_robot.rviz'
    )

    # open the whole urdf_file_name file and read it content to robot_desc
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf_file]
        ),
        Node(
            package='s6_py_urdf',
            executable='rdk_x3_robot_exe',
            name='rdk_x3_robot',
            output='screen'
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_config_file]
        # ),
    ])
