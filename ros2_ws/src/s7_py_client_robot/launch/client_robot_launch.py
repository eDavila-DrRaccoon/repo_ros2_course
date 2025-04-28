import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Path to package
    pkg_share = get_package_share_directory('s7_py_client_robot')

    # Path to URDF file
    robot1_urdf = os.path.join(pkg_share, 'urdf', 'robot1.urdf')

    # Path to RViz configuration file
    rviz_config_file = os.path.join(
        pkg_share,
        'rviz',
        'robot1.rviz'
    )

    # Read URDF files
    with open(robot1_urdf, 'r') as infp:
        robot1_description = infp.read()

    return LaunchDescription([
        # robot1 robot_state_publisher
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     # name='robot1_state_publisher',
        #     namespace='robot1',
        #     parameters=[{'robot_description': robot1_description}],
        #     output='screen'
        # ),
        # robot1 state publisher node
        Node(
            package='s7_py_client_robot',
            executable='client_robot1_exe',
            name='robot1',
            namespace='robot1',
            remappings=[  # <<< this makes /robot1/get_two_poses → /get_two_poses
                ('get_two_poses', '/get_two_poses'),
            ],
            output='screen'
        ),

        # RViz2
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file]
        #     output='screen',
        # )
    ])
