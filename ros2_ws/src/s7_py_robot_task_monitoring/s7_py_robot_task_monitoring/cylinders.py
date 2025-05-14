#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations

class CylinderVisualizer(Node):
    def __init__(self):
        super().__init__('cylinder_visualizer')
        self.get_logger().info("Cylinder Visualizer node has been started")

        # TF broadcaster for all cylinders
        self.broadcaster = TransformBroadcaster(self)

        # Create subscribers for pickup & delivery for robots 1–4
        self.subs = [] # keep subscriptions alive
        for rid in range(1, 2):
            ns = f'/robot{rid}'
            sub_pick = self.create_subscription(
                Pose2D, f'{ns}/pickup_pose',
                lambda msg, r=rid: self.pose_callback(msg, r, 'pickup'),
                10)
            sub_del = self.create_subscription(
                Pose2D, f'{ns}/delivery_pose',
                lambda msg, r=rid: self.pose_callback(msg, r, 'delivery'),
                10)
            self.subs += [sub_pick, sub_del]

    def pose_callback(self, msg: Pose2D, robot_id: int, kind: str):
        """
        Broadcast a TF frame for a cylinder based on:
        - robot_id: 1-4
        - kind: 'pickup' or 'delivery'
        """
        # Determine cylinder index: pickup1→1, delivery1→2, pickup2→3, etc.
        base = (robot_id - 1) * 2
        cyl_idx = base + (1 if kind == 'pickup' else 2)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = f'cylinder{cyl_idx}/base_footprint'

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
        self.get_logger().info(f"Broadcasted cylinder{cyl_idx}: ({msg.x:.2f}, {msg.y:.2f}, {msg.theta:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = CylinderVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
