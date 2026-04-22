from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # AMR: RobotStatus publisher node
    amr_status = Node(
        package='s7_py_client_robot',
        executable='client_amr_exe',
        name='client_amr',
        namespace='rdk_x3_amr',
        remappings=[ # <<< this makes /rdk_x3_amr/get_two_poses → /get_two_poses
            ('get_two_poses', '/get_two_poses'),
        ],
        output='screen'
    )

    # UAV: RobotStatus publisher node
    uav_status = Node(
        package='s7_py_client_robot',
        executable='tello_uav_exe',
        name='tello_uav',
        namespace='tello_uav',
        output='screen'
    )

    return LaunchDescription([
        amr_status,
        uav_status
    ])
