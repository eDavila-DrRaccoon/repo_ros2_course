#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from s7_robot_network_interface.msg import UAVStatus
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import tf_transformations
import math

class StatePublisher(Node):
    def __init__(self):
        super().__init__('py_state_publisher_node')
        self.get_logger().info("Python state publisher node has been started")

        # --- Robot ID ---
        self.robot_id = 2

        # --- Publishers ---
        self.robot_pub = self.create_publisher(UAVStatus, 'robot_status', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.broadcaster = TransformBroadcaster(self)

        # Timer for publishing at ~30Hz
        self.timer = self.create_timer(0.033, self.publish)

        # State variables
        self.degree = math.pi / 180.0 # degree means one degree in radians
        self.spin_angle = 0.0
        self.step = self.degree
        self.angle = 0.0

    def publish(self):
        # --- Publish the current Robot Status ---
        status_msg = UAVStatus()
        status_msg.timestamp = self.get_clock().now().to_msg()
        status_msg.robot_id = self.robot_id
        status_msg.pose.position.x = math.cos(self.angle) * 2.0
        status_msg.pose.position.y = math.sin(self.angle) * 1.0
        status_msg.pose.position.z = math.sin(self.angle - math.pi / 2) * 0.5 + 0.8
        q = tf_transformations.quaternion_from_euler(0, 0, self.angle + math.pi / 2)
        status_msg.pose.orientation.x = q[0]
        status_msg.pose.orientation.y = q[1]
        status_msg.pose.orientation.z = q[2]
        status_msg.pose.orientation.w = q[3]

        # --- Publish joint state ---
        # Create JointState message
        joint_state = JointState()
        # Add time stamp
        joint_state.header.stamp = self.get_clock().now().to_msg()
        # Specify joints' name which are defined in the URDF file and their content
        joint_state.name = ['base_joint']
        joint_state.position = [self.spin_angle]

        # --- Publish transform for RViz ---
        # Create TransformStamped message
        t = TransformStamped()
        # Add time stamp
        t.header.stamp = self.get_clock().now().to_msg()
        # Specify the father and child frames
        # odom is the base coordinate system of tf2
        t.header.frame_id = 'odom'
        # base_footprint is defined in URDF file and it is the base coordinate of model
        t.child_frame_id = 'tello_uav/base_footprint'

        # Add translation change
        t.transform.translation.x = status_msg.pose.position.x
        t.transform.translation.y = status_msg.pose.position.y
        t.transform.translation.z = status_msg.pose.position.z

        # Euler angle into Quaternion and add rotation change
        t.transform.rotation.x = status_msg.pose.orientation.x
        t.transform.rotation.y = status_msg.pose.orientation.y
        t.transform.rotation.z = status_msg.pose.orientation.z
        t.transform.rotation.w = status_msg.pose.orientation.w

        # Update state for next cycle
        self.spin_angle -= self.step
        if self.spin_angle < -math.pi or self.spin_angle > math.pi:
            self.spin_angle *= -1
        self.angle += self.degree # Change the angle at a slow pace

        # --- Publish messages ---
        self.robot_pub.publish(status_msg)
        self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(t)

        self.get_logger().info("Publishing joint state and transform")

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
