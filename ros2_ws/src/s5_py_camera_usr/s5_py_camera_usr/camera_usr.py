#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('py_camera_user_node')
        self.get_logger().info("Python Camera User node has been started")

        # Initialize CvBridge and create a subscriber
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 1)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Display the image
            cv2.imshow('Python Camera User', cv_image)
            cv2.waitKey(1)
            self.main_process(cv_image)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
    
    def main_process(self, image):
        # Get the height and width
        height, width = image.shape[:2]

        # Resize the image
        resized_image = cv2.resize(image, (int(0.5*width), int(0.5*height)))

        # Convert the image to grayscale
        gray_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        
        # Apply histogram equalization
        equalized_image = cv2.equalizeHist(gray_image)

        # create a CLAHE object (Arguments are optional).
        clahe = cv2.createCLAHE(clipLimit=0.5, tileGridSize=(8,8))
        clahe_image = clahe.apply(gray_image)

        # Display the images, stacking them side-by-side
        stacked_img = np.hstack((gray_image, equalized_image, clahe_image))
        cv2.imshow("Grayscale - HE - CLAHE (PY)", stacked_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        node.get_logger().info("All OpenCV windows destroyed.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
