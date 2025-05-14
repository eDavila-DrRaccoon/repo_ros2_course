#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Header
import time

def find_devices():
    for i in range(5): # Check up to 5 devices
        cap_test = cv2.VideoCapture(i)
        if cap_test.isOpened():
            print(f"Device {i} is available.")
        # Release the VideoCapture object
        cap_test.release()

class CameraNode(Node):
    def __init__(self):
        super().__init__('py_camera_node')
        self.get_logger().info("Python Camera node has been started")

        # Check available camera IDs
        # find_devices()
        
        # Declare parameters with default values
        self.declare_parameter('camera.deviceID', 0)
        self.declare_parameter('camera.width', 1280)
        self.declare_parameter('camera.height', 720)

        # Get parameters
        self.deviceID = self.get_parameter('camera.deviceID').get_parameter_value().integer_value
        self.width = self.get_parameter('camera.width').get_parameter_value().integer_value
        self.height = self.get_parameter('camera.height').get_parameter_value().integer_value

        # Open the camera
        self.cap = cv2.VideoCapture(self.deviceID)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            rclpy.shutdown()
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        print(f"Set Frame Width: {self.width}")
        print(f"Set Frame Height: {self.height}")

        ret, frame = self.cap.read()

        # Current cam roperties
        print(f"Current camera properties:")
        print(f"Device ID: {self.deviceID}")
        print(f"Frame Width: {frame.shape[1]}")
        print(f"Frame Height: {frame.shape[0]}")

        # Initialize start time
        self.start_time = time.time()
        self.fps = 0.0

        # Initialize CvBridge and create a publisher
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 1)

        # Start the timer for capturing and publishing images
        self.timer = self.create_timer(0.016, self.CaptureAndPublish)
        self.frame_count = 0

    def CaptureAndPublish(self):
        ret, frame = self.cap.read()
        if ret:
            # Increment frame count
            self.frame_count += 1

            # Calculate FPS every second
            elapsed_time = time.time() - self.start_time
            if elapsed_time >= 1.0:
                self.fps = self.frame_count / elapsed_time
                self.frame_count = 0
                self.start_time = time.time()
            
            # Convert FPS to string
            fps_text = f"FPS: {self.fps:.2f}"

            # Put FPS text on the frame
            cv2.putText(frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            # Convert OpenCV image to ROS message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_message.header = header

            # Publish the image
            self.publisher.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cap.release()
        node.get_logger().info("Camera resource released.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
