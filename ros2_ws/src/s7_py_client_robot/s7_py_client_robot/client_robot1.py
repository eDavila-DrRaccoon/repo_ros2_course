#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from s7_robot_network_interface.srv import GetTwoPoses
from s7_robot_network_interface.msg import RobotStatus
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import math

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher_node')
        self.get_logger().info('Pose Publisher node has been started')

        # --- Service client setup ---
        self.client = self.create_client(GetTwoPoses, 'get_two_poses')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for service. Exiting.')
                return
            self.get_logger().info('Waiting for service to become available...')
        self.req = GetTwoPoses.Request()

        # --- Robot ID ---
        self.robot_id = 1

        # --- Robot state ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.wheel_angle = 0.0

        # --- Movement parameters ---
        self.step_size = 0.01       # meters per update
        self.yaw_step = math.pi / 180.0  # radians per update
        self.wheel_step = math.pi / 180.0  # radians per update

        # --- State machine stage ---
        # 0: rotate to pickup
        # 1: move to pickup
        # 2: rotate to pickup final yaw
        # 3: rotate to delivery
        # 4: move to delivery
        # 5: rotate to delivery final yaw
        # 6: request new poses
        self.stage = 0

        # --- Fetch initial poses ---
        self.get_new_poses()

        # --- Publishers ---
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.robot_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.broadcaster = TransformBroadcaster(self)

        # --- Timer for periodic updates (~30 Hz) ---
        self.timer = self.create_timer(0.033, self.publish)
    
    def get_new_poses(self):
        """Call the get_two_poses service."""
        self.req.robot_id = self.robot_id
        future = self.client.call_async(self.req)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        """Store pickup & delivery poses."""
        resp = future.result()
        if resp is None:
            self.get_logger().error('Failed to call service get_two_poses')
            return

        # Store poses
        self.pickup_pose = resp.pickup_pose
        self.delivery_pose = resp.delivery_pose

        self.pickup_yaw = self.pickup_pose.theta
        self.delivery_yaw = self.delivery_pose.theta

        # Pre-compute yaw needed to face each goal
        self.pickup_to_yaw = math.atan2(
            self.pickup_pose.y - self.y,
            self.pickup_pose.x - self.x
        )

        self.get_logger().info(
            f'New pickup_pose: ({self.pickup_pose.x:.2f}, {self.pickup_pose.y:.2f}, {self.pickup_pose.theta:.2f} rad)' 
        )
        self.get_logger().info(
            f'New delivery_pose: ({self.delivery_pose.x:.2f}, {self.delivery_pose.y:.2f}, {self.delivery_pose.theta:.2f} rad)'  
        )

    def normalize_angle(self, angle: float) -> float:
        """Normalize an angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def reached(self, a: float, b: float, tol: float = 1e-2) -> bool:
        """Return True if a and b are within tol."""
        return abs(a - b) < tol

    def publish(self):
        # --- State machine logic ---
        if self.stage == 0:
            # 1. Rotate toward pickup
            yaw_diff = self.normalize_angle(self.pickup_to_yaw - self.yaw)
            if abs(yaw_diff) > self.yaw_step:
                self.yaw += math.copysign(self.yaw_step, yaw_diff)
                self.yaw = self.normalize_angle(self.yaw)
            else:
                self.yaw = self.pickup_to_yaw
                self.stage = 1

        elif self.stage == 1:
            # 2. Move toward pickup position
            dx = self.pickup_pose.x - self.x
            dy = self.pickup_pose.y - self.y
            dist = math.hypot(dx, dy)
            if dist > self.step_size:
                direction = math.atan2(dy, dx)
                self.x += math.cos(direction) * self.step_size
                self.y += math.sin(direction) * self.step_size
            else:
                self.x = self.pickup_pose.x
                self.y = self.pickup_pose.y
                self.stage = 2

        elif self.stage == 2:
            # 3. Rotate to final pickup yaw
            yaw_diff = self.normalize_angle(self.pickup_yaw - self.yaw)
            if abs(yaw_diff) > self.yaw_step:
                self.yaw += math.copysign(self.yaw_step, yaw_diff)
                self.yaw = self.normalize_angle(self.yaw)
            else:
                self.yaw = self.pickup_yaw
                # Prepare rotation to delivery
                self.delivery_to_yaw = math.atan2(
                    self.delivery_pose.y - self.y,
                    self.delivery_pose.x - self.x
                )
                self.stage = 3

        elif self.stage == 3:
            # 4. Rotate toward delivery
            yaw_diff = self.normalize_angle(self.delivery_to_yaw - self.yaw)
            if abs(yaw_diff) > self.yaw_step:
                self.yaw += math.copysign(self.yaw_step, yaw_diff)
                self.yaw = self.normalize_angle(self.yaw)
            else:
                self.yaw = self.delivery_to_yaw
                self.stage = 4

        elif self.stage == 4:
            # 5. Move toward delivery position
            dx = self.delivery_pose.x - self.x
            dy = self.delivery_pose.y - self.y
            dist = math.hypot(dx, dy)
            if dist > self.step_size:
                direction = math.atan2(dy, dx)
                self.x += math.cos(direction) * self.step_size
                self.y += math.sin(direction) * self.step_size
            else:
                self.x = self.delivery_pose.x
                self.y = self.delivery_pose.y
                self.stage = 5

        elif self.stage == 5:
            # 6. Rotate to final delivery yaw
            yaw_diff = self.normalize_angle(self.delivery_yaw - self.yaw)
            if abs(yaw_diff) > self.yaw_step:
                self.yaw += math.copysign(self.yaw_step, yaw_diff)
                self.yaw = self.normalize_angle(self.yaw)
            else:
                self.yaw = self.delivery_yaw
                self.stage = 6

        elif self.stage == 6:
            # 7. Completed delivery, request new goals
            self.get_logger().info('Completed delivery. Requesting new goals...')
            self.get_new_poses()
            self.stage = 0

        # --- Publish joint states ---
        self.wheel_angle += 4 * self.wheel_step
        if abs(self.wheel_angle) > math.pi:
            self.wheel_angle *= -1

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            # 'base_joint',
            'left_front_joint',
            'left_back_joint',
            'right_front_joint',
            'right_back_joint'
        ]
        js.position = [
            # 0.0,
            self.wheel_angle,
            self.wheel_angle,
            -self.wheel_angle,
            -self.wheel_angle
        ]
        self.joint_pub.publish(js)

        # --- Publish the current Robot Status ---
        robot_msg = RobotStatus()
        robot_msg.timestamp = self.get_clock().now().to_msg()
        robot_msg.robot_id = self.robot_id
        robot_msg.task_stage = self.stage
        robot_msg.pose.x = self.x
        robot_msg.pose.y = self.y
        robot_msg.pose.theta = self.yaw
        robot_msg.align_yaw = True
        self.robot_pub.publish(robot_msg)

        # --- Publish transform for RViz ---
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        namespace = self.get_namespace().strip('/')
        t.child_frame_id = f'{namespace}/base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q_tf = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        t.transform.rotation.x = q_tf[0]
        t.transform.rotation.y = q_tf[1]
        t.transform.rotation.z = q_tf[2]
        t.transform.rotation.w = q_tf[3]
        self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Exception caught: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
