#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from s4_custom_interface.msg import HardwareStatus
from s4_custom_interface.msg import Sphere

class PyPublisher(Node):
    def __init__(self):
        super().__init__('py_publisher_node')
        self.get_logger().info("Python Publisher node has been started")
        self.publisher_ = self.create_publisher(HardwareStatus, 'py_hs_topic', 10)
        self.publisher2_ = self.create_publisher(Sphere, 'py_sphere_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # HardwareStatus message
        msg = HardwareStatus()
        msg.temperature = 45
        msg.are_motors_up = True
        msg.debug_message = "Eduardo's Python system running smoothly"
        msg.the_time = self.get_clock().now().to_msg()

        self.get_logger().info("Publishing HardwareStatus: Temperature: %d, Motors Up: %s, Debug Message: %s, Time: %d.%09d" % (
                    msg.temperature,
                    msg.are_motors_up,
                    msg.debug_message,
                    msg.the_time.sec,
                    msg.the_time.nanosec))

        self.publisher_.publish(msg)

        # Sphere message
        msg2 = Sphere()
        msg2.center.x = 1.0
        msg2.center.y = 2.0
        msg2.center.z = 3.0
        msg2.radius = 4.0

        self.get_logger().info("Publishing Sphere: x: %f, y: %f, z: %f, Radius: %f" % (
                    msg2.center.x,
                    msg2.center.y,
                    msg2.center.z,
                    msg2.radius))

        self.publisher2_.publish(msg2)

def main(args=None):
    rclpy.init(args=args)
    node = PyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()