#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import tf_transformations
import math

class StatePublisher(Node):
    def __init__(self):
        super().__init__('py_state_publisher_node')
        self.get_logger().info("Python state publisher node has been started")

        # Create a publisher to tell robot_state_publisher the JointState information.
        # robot_state_publisher will deal with this transformation
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Create a broadcaster to publish the transform between coordinate frames that
        # will determine the position of coordinate system 'base_footprint' in coordinate system 'odom'
        self.broadcaster = TransformBroadcaster(self)

        # Timer for publishing at ~30Hz
        self.timer = self.create_timer(0.033, self.publish)

        # State variables
        # degree means one degree
        self.degree = math.pi / 180.0
        self.spin_angle = 0.0
        self.step = self.degree
        self.angle = 0.0

    def publish(self):
        # Create JointState message
        joint_state = JointState()
        # Add time stamp
        joint_state.header.stamp = self.get_clock().now().to_msg()
        # Specify joints' name which are defined in the URDF file and their content
        joint_state.name = ['base_joint']
        joint_state.position = [self.spin_angle]

        # Create TransformStamped message
        t = TransformStamped()
        # Add time stamp
        t.header.stamp = self.get_clock().now().to_msg()
        # Specify the father and child frames
        # odom is the base coordinate system of tf2
        t.header.frame_id = 'odom'
        # base_footprint is defined in URDF file and it is the base coordinate of model
        t.child_frame_id = 'wheel_base_footprint'

        # Add translation change
        t.transform.translation.x = math.cos(self.angle) * 1.0
        t.transform.translation.y = math.sin(self.angle) * 1.0
        t.transform.translation.z = 0.0

        # Euler angle into Quaternion and add rotation change
        q = tf_transformations.quaternion_from_euler(0, 0, self.angle + math.pi / 2)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Update state for next cycle
        self.spin_angle -= self.step
        if self.spin_angle < -math.pi or self.spin_angle > math.pi:
            self.spin_angle *= -1

        self.angle += self.degree # Change the angle at a slow pace

        # Publish messages
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
