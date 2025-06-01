import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to package
    pkg_share = get_package_share_directory('s7_py_robot_task_monitoring')

    # Paths to URDF files
    cylinder1_urdf = os.path.join(pkg_share, 'urdf', 'cylinder1.urdf')
    cylinder2_urdf = os.path.join(pkg_share, 'urdf', 'cylinder2.urdf')
    cylinder3_urdf = os.path.join(pkg_share, 'urdf', 'cylinder3.urdf')
    cylinder4_urdf = os.path.join(pkg_share, 'urdf', 'cylinder4.urdf')
    cylinder5_urdf = os.path.join(pkg_share, 'urdf', 'cylinder5.urdf')
    cylinder6_urdf = os.path.join(pkg_share, 'urdf', 'cylinder6.urdf')
    cylinder7_urdf = os.path.join(pkg_share, 'urdf', 'cylinder7.urdf')
    cylinder8_urdf = os.path.join(pkg_share, 'urdf', 'cylinder8.urdf')
    
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
    with open(cylinder1_urdf, 'r') as infp:
        cylinder1_description = infp.read()
    
    with open(cylinder2_urdf, 'r') as infp:
        cylinder2_description = infp.read()
    
    with open(cylinder3_urdf, 'r') as infp:
        cylinder3_description = infp.read()
    
    with open(cylinder4_urdf, 'r') as infp:
        cylinder4_description = infp.read()
    
    with open(cylinder5_urdf, 'r') as infp:
        cylinder5_description = infp.read()
    
    with open(cylinder6_urdf, 'r') as infp:
        cylinder6_description = infp.read()
    
    with open(cylinder7_urdf, 'r') as infp:
        cylinder7_description = infp.read()
    
    with open(cylinder8_urdf, 'r') as infp:
        cylinder8_description = infp.read()
    
    with open(robot1_urdf, 'r') as infp:
        robot1_description = infp.read()
    
    with open(robot2_urdf, 'r') as infp:
        robot2_description = infp.read()
    
    with open(robot3_urdf, 'r') as infp:
        robot3_description = infp.read()
    
    with open(robot4_urdf, 'r') as infp:
        robot4_description = infp.read()
    
    # cylinder1 (pickup pose for Robot1) robot_state_publisher
    cylinder1_robot1_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='cylinder1_state_publisher',
        namespace='cylinder1',
        parameters=[{'robot_description': cylinder1_description}],
        output='screen'
    )
    # cylinder2 (delivery pose for Robot1) robot_state_publisher
    cylinder2_robot1_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='cylinder2_state_publisher',
        namespace='cylinder2',
        parameters=[{'robot_description': cylinder2_description}],
        output='screen'
    )

    # cylinder3 (pickup pose for Robot2) robot_state_publisher
    cylinder3_robot2_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='cylinder3_state_publisher',
        namespace='cylinder3',
        parameters=[{'robot_description': cylinder3_description}],
        output='screen'
    )
    # cylinder4 (delivery pose for Robot2) robot_state_publisher
    cylinder4_robot2_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='cylinder4_state_publisher',
        namespace='cylinder4',
        parameters=[{'robot_description': cylinder4_description}],
        output='screen'
    )

    # cylinder5 (pickup pose for Robot3) robot_state_publisher
    cylinder5_robot3_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='cylinder5_state_publisher',
        namespace='cylinder5',
        parameters=[{'robot_description': cylinder5_description}],
        output='screen'
    )
    # cylinder6 (delivery pose for Robot3) robot_state_publisher
    cylinder6_robot3_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='cylinder6_state_publisher',
        namespace='cylinder6',
        parameters=[{'robot_description': cylinder6_description}],
        output='screen'
    )

    # cylinder7 (pickup pose for Robot4) robot_state_publisher
    cylinder7_robot4_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='cylinder7_state_publisher',
        namespace='cylinder7',
        parameters=[{'robot_description': cylinder7_description}],
        output='screen'
    )
    # cylinder8 (delivery pose for Robot4) robot_state_publisher
    cylinder8_robot4_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='cylinder8_state_publisher',
        namespace='cylinder8',
        parameters=[{'robot_description': cylinder8_description}],
        output='screen'
    )

    # cylinders pose subscriber-publisher node
    cylinders_tf = Node(
        package='s7_py_robot_task_monitoring',
        executable='cylinders_exe',
        # name='cylinders_exe',
        output='screen'
    )

    # robot1 robot_state_publisher
    robot1_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='robot1_state_publisher',
        namespace='robot1',
        parameters=[{'robot_description': robot1_description}],
        output='screen'
    )
    # robot2 robot_state_publisher
    robot2_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='robot2_state_publisher',
        namespace='robot2',
        parameters=[{'robot_description': robot2_description}],
        output='screen'
    )
    # robot3 robot_state_publisher
    robot3_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='robot3_state_publisher',
        namespace='robot3',
        parameters=[{'robot_description': robot3_description}],
        output='screen'
    )
    # robot4 robot_state_publisher
    robot4_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='robot4_state_publisher',
        namespace='robot4',
        parameters=[{'robot_description': robot4_description}],
        output='screen'
    )

    # robots pose subscriber-publisher node
    robots_tf = Node(
        package='s7_py_robot_task_monitoring',
        executable='robots_exe',
        name='robots_exe',
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
        cylinder1_robot1_desc,
        cylinder2_robot1_desc,
        robot1_desc,

        cylinder3_robot2_desc,
        cylinder4_robot2_desc,
        robot2_desc,

        cylinder5_robot3_desc,
        cylinder6_robot3_desc,
        robot3_desc,

        cylinder7_robot4_desc,
        cylinder8_robot4_desc,
        robot4_desc,

        cylinders_tf,
        robots_tf,
        # rviz
    ])
