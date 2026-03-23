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
        'client_robot.rviz'
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
    
    # robot1: RobotStatus publisher node
    robotstatus1 = Node(
        package='s7_py_client_robot',
        executable='client_robot1_exe',
        name='robot1',
        namespace='robot1',
        remappings=[  # <<< this makes /robot1/get_two_poses → /get_two_poses
            ('get_two_poses', '/get_two_poses'),
        ],
        output='screen'
    )

    # robot2: RobotStatus publisher node
    robotstatus2 = Node(
        package='s7_py_client_robot',
        executable='client_robot2_exe',
        name='robot2',
        namespace='robot2',
        remappings=[  # <<< this makes /robot1/get_two_poses → /get_two_poses
            ('get_two_poses', '/get_two_poses'),
        ],
        output='screen'
    )

    # robot3: RobotStatus publisher node
    robotstatus3 = Node(
        package='s7_py_client_robot',
        executable='client_robot3_exe',
        name='robot3',
        namespace='robot3',
        remappings=[  # <<< this makes /robot1/get_two_poses → /get_two_poses
            ('get_two_poses', '/get_two_poses'),
        ],
        output='screen'
    )

    # robot4: RobotStatus publisher node
    robotstatus4 = Node(
        package='s7_py_client_robot',
        executable='client_robot4_exe',
        name='robot4',
        namespace='robot4',
        remappings=[  # <<< this makes /robot1/get_two_poses → /get_two_poses
            ('get_two_poses', '/get_two_poses'),
        ],
        output='screen'
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        robotstatus1,
        # robotstatus2,
        # robotstatus3,
        # robotstatus4,
        # rviz,
    ])
