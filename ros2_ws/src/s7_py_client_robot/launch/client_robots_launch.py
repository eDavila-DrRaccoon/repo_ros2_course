import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to package
    pkg_share = get_package_share_directory('s7_py_client_robot')

    # Paths to URDF files
    robot1_urdf = os.path.join(pkg_share, 'urdf', 'robot1.urdf')
    robot2_urdf = os.path.join(pkg_share, 'urdf', 'robot2.urdf')
    robot3_urdf = os.path.join(pkg_share, 'urdf', 'robot3.urdf')
    robot4_urdf = os.path.join(pkg_share, 'urdf', 'robot4.urdf')

    # Path to RViz configuration file
    rviz_config_file = os.path.join(
        pkg_share,
        'rviz',
        'robot1.rviz'
    )

    # Read URDF files
    with open(robot1_urdf, 'r') as infp:
        robot1_description = infp.read()
    
    with open(robot2_urdf, 'r') as infp:
        robot2_description = infp.read()
    
    with open(robot3_urdf, 'r') as infp:
        robot3_description = infp.read()
    
    with open(robot4_urdf, 'r') as infp:
        robot4_description = infp.read()

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

        # # robot2 robot_state_publisher
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot2_state_publisher',
        #     namespace='robot2',
        #     parameters=[
        #         {'robot_description': robot2_description},
        #     ],
        #     output='screen'
        # ),
        # robot2 state publisher node
        # Node(
        #     package='s7_py_client_robot',
        #     executable='client_robot2_exe',
        #     name='robot2',
        #     namespace='robot2',
        #     remappings=[  # <<< this makes /robot1/get_two_poses → /get_two_poses
        #         ('get_two_poses', '/get_two_poses'),
        #     ],
        #     output='screen'
        # ),

        # # robot3 robot_state_publisher
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot3_state_publisher',
        #     namespace='robot3',
        #     parameters=[
        #         {'robot_description': robot3_description},
        #     ],
        #     output='screen'
        # ),
        # robot3 state publisher node
        # Node(
        #     package='s7_py_client_robot',
        #     executable='client_robot3_exe',
        #     name='robot3',
        #     namespace='robot3',
        #     remappings=[  # <<< this makes /robot1/get_two_poses → /get_two_poses
        #         ('get_two_poses', '/get_two_poses'),
        #     ],
        #     output='screen'
        # ),

        # # robot4 robot_state_publisher
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot4_state_publisher',
        #     namespace='robot4',
        #     parameters=[
        #         {'robot_description': robot4_description},
        #     ],
        #     output='screen'
        # ),
        # robot4 state publisher node
        # Node(
        #     package='s7_py_client_robot',
        #     executable='client_robot4_exe',
        #     name='robot4',
        #     namespace='robot4',
        #     remappings=[  # <<< this makes /robot1/get_two_poses → /get_two_poses
        #         ('get_two_poses', '/get_two_poses'),
        #     ],
        #     output='screen'
        # ),

        # RViz2
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file]
        #     output='screen',
        # )
    ])
