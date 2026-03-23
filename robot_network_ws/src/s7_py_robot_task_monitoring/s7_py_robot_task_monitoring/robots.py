#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from s7_robot_network_interface.msg import RobotStatus
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import math

class RobotVisualizer(Node):
    def __init__(self):
        super().__init__('robot_visualizer')
        self.get_logger().info("Robot Visualizer node has been started")

        # --- Robot wheel state ---
        self.wheel_angle = 0.0

        # --- Movement parameters ---
        self.wheel_step = math.pi / 180.0  # radians per update

        # --- Number of robots ---
        self.robot_num = 4
        
        # Create subscribers to robot_status topic for each robot
        self.subs = [] # keep subscriptions alive
        for rid in range(1, self.robot_num + 1):
            topic = f'/robot{rid}/robot_status'
            # Subscribe to '/robotN/robot_status'
            sub = self.create_subscription(
                RobotStatus, topic,
                lambda msg, r=rid: self.RobotPose_callback(msg, r),
                10)
            self.subs.append(sub)
        
        # Create publishers to joint_states topic for each robot
        self.pubs = [] # keep publications alive
        for rid in range(1, self.robot_num + 1):
            topic = f'/robot{rid}/joint_states'
            # Publish to '/robotN/joint_states'
            pub = self.create_publisher(JointState, topic, 10)
            self.pubs.append(pub)

        # --- Timer for periodic updates (~30 Hz) ---
        self.timer = self.create_timer(0.033, self.publish)

        # TF broadcaster for all robots
        self.broadcaster = TransformBroadcaster(self)

    def RobotPose_callback(self, msg: RobotStatus, robot_id: int):
        """
        Broadcast 'odom'→'robotN/base_footprint' transform
        based on Pose2D field from RobotStatus message.
        """
        t = TransformStamped()
        t.header.stamp = msg.timestamp # self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = f'robot{robot_id}/base_footprint'

        # Translation from Pose2D
        t.transform.translation.x = msg.pose.x
        t.transform.translation.y = msg.pose.y
        t.transform.translation.z = 0.0

        # Rotation from yaw θ
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, msg.pose.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Publish transform to tf2
        self.broadcaster.sendTransform(t)
        self.get_logger().debug(
            f"Broadcasted robot{robot_id} at ({msg.pose.x:.2f}, {msg.pose.y:.2f}, {msg.pose.theta:.2f})"
        )
    
    def publish(self):
        # --- Publish joint states ---
        for robot_id in range(1, self.robot_num + 1):
            self.wheel_angle += 1 * self.wheel_step
            if abs(self.wheel_angle) > math.pi:
                self.wheel_angle *= -1

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [
                'left_front_joint',
                'left_back_joint',
                'right_front_joint',
                'right_back_joint'
            ]
            js.position = [
                self.wheel_angle,
                self.wheel_angle,
                -self.wheel_angle,
                -self.wheel_angle
            ]
            self.pubs[robot_id-1].publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = RobotVisualizer()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Exception caught: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
