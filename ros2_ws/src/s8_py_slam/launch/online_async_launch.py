import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Path to package
    pkg_share = get_package_share_directory('s8_py_slam')

    # Path to SLAM Toolbox parameters file
    slam_params_file = os.path.join(pkg_share, 'config',  'mapper_params_online_async.yaml')

    return LaunchDescription([
        # 3) Slam Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='async_slam_toolbox',
            parameters=[
                slam_params_file,
                {'use_sim_time': True}
                ],
            output='screen'
        )
    ])
