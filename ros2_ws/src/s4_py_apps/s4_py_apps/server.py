#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from s4_custom_interface.srv import AddThreeInts

class PyService(Node):
    def __init__(self):
        super().__init__('py_server_node')
        self.get_logger().info("Python Server node has been started")
        self.srv = self.create_service(AddThreeInts, 'py_add_three_ints_service', self.AddThreeInts_callback)
        self.get_logger().info("Service 'py_add_three_ints_service' is ready to receive requests")

    def AddThreeInts_callback(self, request, response):
        response.sum = request.a + request.b + request.c
        self.get_logger().info('Received request:\na: %d, b: %d, b: %d' % (request.a, request.b, request.c))
        self.get_logger().info('Sending back response: sum = %d' % (response.sum))
        return response

def main(args=None):
    rclpy.init(args=args)
    py_service = PyService()
    try:
        rclpy.spin(py_service)
    except:
        py_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()