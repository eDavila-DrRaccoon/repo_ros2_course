#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class PyService(Node):

    def __init__(self):
        super().__init__('py_server_node')
        self.get_logger().info("Python Server node has been started")
        self.srv = self.create_service(AddTwoInts, 'py_add_two_ints_service', self.AddTwoInts_callback)
        self.get_logger().info("Service 'py_add_two_ints_service' is ready to receive requests")

    def AddTwoInts_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Received request:\na: %d, b: %d' % (request.a, request.b))
        self.get_logger().info('Sending back response: sum = %d' % (response.sum))

        return response

def main(args=None):
    rclpy.init(args=args)
    node = PyService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()