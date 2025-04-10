import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the YAML file
    params_file = os.path.join(
        get_package_share_directory('s5_cpp_camera'),
        'config',
        'camera_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='s5_cpp_camera',
            name='cpp_camera_node',
            executable='cpp_camera_exe',
            parameters=[params_file],
            output = 'screen'
        )
    ])
