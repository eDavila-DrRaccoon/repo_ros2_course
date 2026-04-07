#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from s4_custom_interface.msg import HardwareStatus
from s4_custom_interface.msg import Sphere

class PySubscriber(Node):
    def __init__(self):
        super().__init__('py_subscriber_node')
        self.get_logger().info("Python Subscriber node has been started")
        self.subscription = self.create_subscription(
            HardwareStatus, 'cpp_hs_topic', self.listener_callback, 10)
        self.subscription2 = self.create_subscription(
            Sphere, 'py_sphere_topic', self.sphere_callback, 10)
        self.subscription  # prevent unused variable warning
        self.subscription2 # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("Received HardwareStatus: Temperature: %d, Motors Up: %s, Debug Message: %s, Time: %d.%09d" % (
                    msg.temperature,
                    msg.are_motors_up,
                    msg.debug_message,
                    msg.the_time.sec,
                    msg.the_time.nanosec))
    
    def sphere_callback(self, msg):
        self.get_logger().info("Received Sphere: x: %f, y: %f, z: %f, Radius: %f" % (
                    msg.center.x,
                    msg.center.y,
                    msg.center.z,
                    msg.radius))

def main(args=None):
    rclpy.init(args=args)
    node = PySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()