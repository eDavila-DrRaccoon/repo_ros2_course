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
    amr_urdf = os.path.join(pkg_share, 'urdf', 'rdk_x3_amr.urdf')
    uav_urdf = os.path.join(pkg_share, 'urdf', 'tello_uav.urdf')

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
    
    with open(amr_urdf, 'r') as infp:
        amr_description = infp.read()
    
    with open(uav_urdf, 'r') as infp:
        uav_description = infp.read()
    
    # cylinder1 (pickup pose for the AMR) robot_state_publisher
    cylinder1_amr_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='cylinder1_state_publisher',
        namespace='cylinder1',
        parameters=[{'robot_description': cylinder1_description}],
        output='screen'
    )
    # cylinder2 (delivery pose for the AMR) robot_state_publisher
    cylinder2_amr_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='cylinder2_state_publisher',
        namespace='cylinder2',
        parameters=[{'robot_description': cylinder2_description}],
        output='screen'
    )

    # cylinders pose subscriber-publisher node
    cylinders_tf = Node(
        package='s7_py_robot_task_monitoring',
        executable='cylinders_exe',
        # name='cylinders_exe',
        output='screen'
    )

    # amr robot_state_publisher
    amr_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='amr_state_publisher',
        namespace='rdk_x3_amr',
        parameters=[{'robot_description': amr_description}],
        output='screen'
    )

    # uav robot_state_publisher
    uav_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='uav_state_publisher',
        namespace='tello_uav',
        parameters=[{'robot_description': uav_description}],
        output='screen'
    )

    # robot (amr) pose subscriber-publisher node
    robot_tf = Node(
        package='s7_py_robot_task_monitoring',
        executable='robot_exe',
        name='robot',
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
        cylinder1_amr_desc,
        cylinder2_amr_desc,
        
        amr_desc,
        uav_desc,

        cylinders_tf,
        robot_tf,
        
        # rviz
    ])
