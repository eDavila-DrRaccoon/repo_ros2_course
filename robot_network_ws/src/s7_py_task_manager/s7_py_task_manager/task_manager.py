#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from s7_robot_network_interface.srv import GetTwoPoses
from geometry_msgs.msg import Pose2D
import random
import math

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
        
        # Define the list of bays (pickup and delivery poses)
        self.bays = [
            (1.91, 3.70, round(-math.pi/2, 2)), # bay1 - (1.89, 3.69, round(-math.pi/2, 2)) for sim_tec_warehouse.yaml
            (0.15, 2.99, 0), # bay2 - (0.15, 3.03, 0) for tec_warehouse.yaml
            (3.72, 2.99, round(math.pi, 2)), # bay3 - (3.72, 3.03, round(math.pi, 2)) for tec_warehouse.yaml
            (0.15, 0.99, 0), # bay4 - (0.15, 0.97, 0) for tec_warehouse.yaml
            (1.16, 0.15, round(math.pi/2, 2)), # bay5 - (1.18, 0.15, round(math.pi/2, 2)) for tec_warehouse.yaml
            (2.30, 0.15, round(math.pi/2, 2)), # bay6 - (2.32, 0.15, round(math.pi/2, 2)) for tec_warehouse.yaml
            (3.29, 0.15, round(math.pi/2, 2)) # bay7 - (3.35, 0.15, round(math.pi/2, 2)) for tec_warehouse.yaml
        ]

    def GetTwoPoses_callback(self, request, response):
        # Randomly select two distinct bays
        pickup_bay, delivery_bay = random.sample(self.bays, 2)

        # Create Pose2D messages
        pickup_pose = Pose2D()
        pickup_pose.x = pickup_bay[0]
        pickup_pose.y = pickup_bay[1]
        pickup_pose.theta = float(pickup_bay[2])

        delivery_pose = Pose2D()
        delivery_pose.x = delivery_bay[0]
        delivery_pose.y = delivery_bay[1]
        delivery_pose.theta = float(delivery_bay[2])

        # Fill response
        response.pickup_pose = pickup_pose
        response.delivery_pose = delivery_pose

        self.get_logger().info(f'Received request from ID: {request.robot_id}')
        self.get_logger().info(f'Providing pickup pose: {pickup_pose}')
        self.get_logger().info(f'Providing delivery pose: {delivery_pose}')

        # Publish to the robot-specific topics
        robot_id = request.robot_id
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


