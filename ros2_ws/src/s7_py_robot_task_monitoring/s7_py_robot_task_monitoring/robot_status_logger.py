#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from s7_robot_network_interface.msg import UAVStatus
from s7_robot_network_interface.msg import AMRStatus
from datetime import datetime, timezone
try:
    from zoneinfo import ZoneInfo  # Available in Python 3.9+ (For Humble/Ubuntu 22.04 with Python 3.10)
except ImportError:
    from backports.zoneinfo import ZoneInfo  # For Python < 3.9 (For Foxy/Ubuntu 20.04 with Python 3.8)
import tf_transformations
import math

class RobotStatusLogger(Node):
    def __init__(self):
        super().__init__('robot_status_logger')
        self.get_logger().info("RobotStatusLogger node has been started")

        # --- Number of robots to be monitored ---
        self.robot_num = 2  # 0: UAV, 1: AMR

        # Robot ID → ID & color label map
        self.rid_label = {
            1: "1 - AMR",
            2: "2 - UAV",
        }

        # Stage → human-readable description map
        self.stage_desc = {
            0: "Rotating to pickup point",
            1: "Moving to pickup point",
            2: "Rotating to pickup final yaw",
            3: "Rotating to delivery point",
            4: "Moving to delivery point",
            5: "Rotating to delivery final yaw",
            6: "Requesting new points",
            7: "Fault: stuck or error",
            8: "No data received",
            9: "Hovering"
        }

        # Threshold for stale data (cycles)
        self.STALE_THRESHOLD = 30

        # Default “no data” template
        self.NO_DATA_STAGE = 8
        self.NO_DATA_ENTRY = lambda rid: {
            'id': self.rid_label[rid],
            'time': '--:--:--.--',
            'align': 'N/A',
            'pose': '(  -.--,  -.--,   -.--°)',
            'stage': rid * 0 + self.NO_DATA_STAGE,  # always 8
            'desc': self.stage_desc[self.NO_DATA_STAGE]
        }

        # 1. Initialize statuses and stale counters
        self.statuses = {rid: self.NO_DATA_ENTRY(rid) for rid in range(1, self.robot_num + 1)}
        self.no_update_counts = {rid: 0 for rid in range(1, self.robot_num + 1)}

        # 2. Subscribe to /rdk_x3_amr/robot_status
        # self.subs = []
        # for rid in range(1, self.robot_num + 1):
        #     topic = '/rdk_x3_amr/robot_status' #f'/robot{rid}/robot_status'
        #     sub = self.create_subscription(
        #         AMRStatus, topic,
        #         lambda msg, r=rid: self.status_callback(msg, r),
        #         10
        #     )
        #     self.subs.append(sub)
        
        # 2. Subscribe to /rdk_x3_amr/robot_status
        self.create_subscription(AMRStatus, '/rdk_x3_amr/robot_status',
            lambda msg: self.status_callback(msg, 1), 10)
        
        # 3. Subscribe to /tello_uav/robot_status
        self.create_subscription(UAVStatus, '/tello_uav/robot_status',
            lambda msg: self.status_callback(msg, 2), 10)

        # 4. Timer to print periodic updates (~30 Hz)
        self.create_timer(0.033, self.print_table)

    def status_callback(self, msg, robot_id: int):
        # 1. Format timestamp as HH:MM:SS.ss
        ts = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9
        dt = datetime.fromtimestamp(ts, tz=timezone.utc).astimezone(ZoneInfo("America/Mexico_City"))
        timestamp_str = dt.strftime('%H:%M:%S.%f')[:-4]  # two decimals

        # 2. Alignment?
        if hasattr(msg, 'align_yaw'):
            align_str = "True" if msg.align_yaw else "False"
        else:
            align_str = 'N/A'

        # 3. Pose
        if hasattr(msg.pose, 'position') and hasattr(msg.pose, 'orientation'):
            # Pose3D as (x,y,z, roll°, pitch°, yaw°)
            x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
            # Get roll, pitch, yaw in degrees
            euler = tf_transformations.euler_from_quaternion((qx, qy, qz, qw))
            roll_deg, pitch_deg, yaw_deg = math.degrees(euler[0]), math.degrees(euler[1]), math.degrees(euler[2])
            pose_str = f"({x:6.2f},{y:6.2f},{z:6.2f},{roll_deg:7.1f}°,{pitch_deg:7.1f}°,{yaw_deg:7.1f}°)"
        else:
            # Pose2D as (x,y,θ°)
            x, y = msg.pose.x, msg.pose.y
            theta_deg = math.degrees(msg.pose.theta)
            pose_str = f"({x:6.2f},{y:6.2f},{theta_deg:7.1f}°)"

        # 4. Stage and description
        if hasattr(msg, 'task_stage'):
            stage = msg.task_stage
            desc = self.stage_desc.get(stage, "Unknown stage")
        else:
            stage = 9
            desc = self.stage_desc.get(stage, "Unknown stage")

        # Update status entry
        self.statuses[robot_id] = {
            'id': self.rid_label.get(robot_id),
            'time': timestamp_str,
            'align': align_str,
            'pose': pose_str,
            'stage': stage,
            'desc': desc
        }

        # Reset stale counter
        self.no_update_counts[robot_id] = 0

    def print_table(self):
        # 0. Record the local “snapshot” time
        now_local = datetime.now(ZoneInfo("America/Mexico_City"))
        snap_str = now_local.strftime('%Y-%m-%d %H:%M:%S.%f')[:-4]

        # 1. Emit a line showing when we took this snapshot
        header = f"--- Robot status snapshot at local time: {snap_str} ---"
        lines = [f"{header:^112s}"]

        # 2. Fields
        fields = "| Robot ID |  Timestamp  |     Pose2D (x,y,θ°) / Pose3D (x,y,z,r°,p°,y°)     | Alignment? | Task Stage |        Task Description        |"
        lines.append(fields)

        # 3. Rows for robots 1–4, guaranteed presence
        for rid in sorted(self.statuses):
            row = self.statuses[rid]
            line = (
                f"| {row['id']:<8s} | {row['time']:^11s} | {row['pose']:<49s} "
                f"| {row['align']:^10s} | {row['stage']:^10d} | {row['desc']:<30s} |"
            )
            lines.append(line)
        
        # 4. Print once
        self.get_logger().info("\n" + "\n".join(lines))

        # 5. Increment counters for robots without updates
        for rid in self.no_update_counts:
            self.no_update_counts[rid] += 1
            # If a robot has gone stale, reset to NO_DATA
            if self.no_update_counts[rid] >= self.STALE_THRESHOLD:
                self.statuses[rid] = self.NO_DATA_ENTRY(rid)
                self.no_update_counts[rid] = 0

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusLogger()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Exception caught: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
