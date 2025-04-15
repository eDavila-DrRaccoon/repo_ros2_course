#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class PyClientAsync(Node):

    def __init__(self):
        super().__init__('py_client_async_node')
        self.get_logger().info("Python Client node has been started")
        self.client = self.create_client(AddTwoInts, 'py_add_two_ints_service')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                return
            self.get_logger().info('Waiting for service to become available...')

        # Prepare a persistent request object
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        print("Usage: client_exe a b")
        sys.exit(1)
    node = PyClientAsync()
    future = node.send_request(int(sys.argv[1]), int(sys.argv[2]))
    # Spin until the future is complete
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if result is not None:
        node.get_logger().info(
            'Received response: %d + %d = %d' %
            (int(sys.argv[1]), int(sys.argv[2]), result.sum)
        )
    else:
        node.get_logger().error('Failed to call service add_two_ints')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()