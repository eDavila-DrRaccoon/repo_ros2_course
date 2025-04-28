#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations

class RobotVisualizer(Node):
    def __init__(self):
        super().__init__('robot_visualizer')
        self.get_logger().info("Robot Visualizer node has been started")

        # TF broadcaster for all robots
        self.broadcaster = TransformBroadcaster(self)

        # Create subscribes for each robot's 2D pose topic
        self.subs = [] # keep subscriptions alive
        for rid in range(1, 2):
            ns = f'/robot{rid}'
            # Subscribe to '/robotN/pose2d'
            sub = self.create_subscription(
                Pose2D, f'{ns}/pose2d',
                lambda msg, r=rid: self.pose_callback(msg, r),
                10)
            self.subs.append(sub)

    def pose_callback(self, msg: Pose2D, robot_id: int):
        """
        Broadcast 'odom'→'robotN/base_footprint' transform
        based on Pose2D message.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = f'robot{robot_id}/base_footprint'

        # Translation from Pose2D
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # Rotation from yaw θ
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Publish to tf2
        self.broadcaster.sendTransform(t)
        self.get_logger().debug(
            f"Broadcasted robot{robot_id} at ({msg.x:.2f}, {msg.y:.2f}, {msg.theta:.2f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = RobotVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
