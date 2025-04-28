import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Path to package
    pkg_share = get_package_share_directory('s7_py_robot_task_visualization')

    # Paths to URDF files
    cylinder1_urdf = os.path.join(pkg_share, 'urdf', 'cylinder1.urdf')
    cylinder2_urdf = os.path.join(pkg_share, 'urdf', 'cylinder2.urdf')
    
    robot1_urdf = os.path.join(pkg_share, 'urdf', 'robot1.urdf')

    # Path to RViz configuration file
    rviz_config_file = os.path.join(
        pkg_share,
        'rviz',
        'robot1.rviz'
    )

    # Read URDF files
    with open(cylinder1_urdf, 'r') as infp:
        cylinder1_description = infp.read()
    
    with open(cylinder2_urdf, 'r') as infp:
        cylinder2_description = infp.read()
    
    with open(robot1_urdf, 'r') as infp:
        robot1_description = infp.read()

    return LaunchDescription([
        # cylinder1 (pickup pose for Robot1) robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            # name='cylinder1_state_publisher',
            namespace='cylinder1',
            parameters=[{'robot_description': cylinder1_description}],
            output='screen'
        ),
        # cylinder2 (delivery pose for Robot1) robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            # name='cylinder2_state_publisher',
            namespace='cylinder2',
            parameters=[{'robot_description': cylinder2_description}],
            output='screen'
        ),

        # cylinders pose subscriber-publisher node
        Node(
            package='s7_py_robot_task_visualization',
            executable='cylinders_exe',
            # name='cylinders_exe',
            output='screen'
        ),

        # robot1 robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            # name='robot1_state_publisher',
            namespace='robot1',
            parameters=[{'robot_description': robot1_description}],
            output='screen'
        ),

        # cylinders pose subscriber-publisher node
        Node(
            package='s7_py_robot_task_visualization',
            executable='robots_exe',
            # name='robots_exe',
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
