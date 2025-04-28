#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from s7_task_manager_interface.srv import GetTwoPoses
from geometry_msgs.msg import Pose2D
import random

class TaskManagerService(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.get_logger().info("Task Manager node has been started")

        # Create service
        self.srv = self.create_service(GetTwoPoses, 'get_two_poses', self.GetTwoPoses_callback)
        self.get_logger().info("Service 'get_two_poses' is ready to receive requests")

        # Create publishers for each robot ID (1-4)
        self.pickup_publishers = {}
        self.delivery_publishers = {}
        for robot_id in range(1, 5):
            ns = f'/robot{robot_id}'
            self.pickup_publishers[robot_id] = self.create_publisher(Pose2D, f'{ns}/pickup_pose', 10)
            self.delivery_publishers[robot_id] = self.create_publisher(Pose2D, f'{ns}/delivery_pose', 10)


    def GetTwoPoses_callback(self, request, response):
        lim = 2.0

        # Generate random pickup pose
        pickup_pose = Pose2D()
        pickup_pose.x = round(random.uniform(-lim, lim), 2)
        pickup_pose.y = round(random.uniform(-lim, lim), 2)
        pickup_pose.theta = round(random.uniform(-3.14, 3.14), 2)

        # Generate random delivery pose
        delivery_pose = Pose2D()
        delivery_pose.x = round(random.uniform(-lim, lim), 2)
        delivery_pose.y = round(random.uniform(-lim, lim), 2)
        delivery_pose.theta = round(random.uniform(-3.14, 3.14), 2)

        # Fill response
        response.pickup_pose = pickup_pose
        response.delivery_pose = delivery_pose

        self.get_logger().info(f'Received request from ID: {request.id}')
        self.get_logger().info(f'Providing pickup pose: {pickup_pose}')
        self.get_logger().info(f'Providing delivery pose: {delivery_pose}')

        # Publish to the robot-specific topics
        robot_id = request.id
        if robot_id in self.pickup_publishers:
            self.pickup_publishers[robot_id].publish(pickup_pose)
            self.delivery_publishers[robot_id].publish(delivery_pose)
        else:
            self.get_logger().warn(f"No publishers set up for robot ID {robot_id}")

        return response

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerService()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Exception caught: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


