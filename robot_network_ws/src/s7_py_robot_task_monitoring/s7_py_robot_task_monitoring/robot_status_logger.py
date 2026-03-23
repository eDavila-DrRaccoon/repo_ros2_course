#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from s7_robot_network_interface.msg import RobotStatus
from datetime import datetime, timezone
try:
    from zoneinfo import ZoneInfo  # Available in Python 3.9+ (For Foxy/Ubuntu 22.04 with Python 3.10)
except ImportError:
    from backports.zoneinfo import ZoneInfo  # For Python < 3.9 (For Humble/Ubuntu 20.04 with Python 3.8)
import math

class RobotStatusLogger(Node):
    def __init__(self):
        super().__init__('robot_status_logger')
        self.get_logger().info("RobotStatusLogger node has been started")

        # Robot ID → ID & color label map
        self.rid_label = {
            1: "1 - Blue",
            2: "2 - Green",
            3: "3 - Pink",
            4: "4 - Violet",
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
            7: "Waiting for connection...",
            8: "No data received"
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

        # Initialize statuses and stale counters
        self.statuses = {rid: self.NO_DATA_ENTRY(rid) for rid in range(1, 5)}
        self.no_update_counts = {rid: 0 for rid in range(1, 5)}

        # 2. Subscribe to /robotN/robot_status for N=1..4
        self.subs = []
        for rid in range(1, 5):
            topic = f'/robot{rid}/robot_status'
            sub = self.create_subscription(
                RobotStatus, topic,
                lambda msg, r=rid: self.status_callback(msg, r),
                10
            )
            self.subs.append(sub)
        
        # 3. Timer to print periodic updates (~30 Hz)
        self.create_timer(0.033, self.print_table)


    def status_callback(self, msg: RobotStatus, robot_id: int):
        # 1. Format timestamp as HH:MM:SS.ss
        ts = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9
        dt = datetime.fromtimestamp(ts, tz=timezone.utc).astimezone(ZoneInfo("America/Mexico_City"))
        timestamp_str = dt.strftime('%H:%M:%S.%f')[:-4]  # two decimals

        # 2. Alignment?
        align_str = "True" if msg.align_yaw else "False"

        # 3. Pose2D as (x,y,θ°)
        x, y = msg.pose.x, msg.pose.y
        theta_deg = math.degrees(msg.pose.theta)
        pose_str = f"({x:6.2f},{y:6.2f},{theta_deg:7.2f}°)"

        # 4. Stage and description
        stage = msg.task_stage
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
        fields = "|  Robot ID  |  Timestamp  | Alignment? |     Pose2D (x,y,θ°)      | Task Stage |        Task Description        |"
        lines.append(fields)

        # 3. Rows for robots 1–4, guaranteed presence
        for rid in sorted(self.statuses):
            row = self.statuses[rid]
            line = (
                f"| {row['id']:<10s} | {row['time']:^11s} | {row['align']:^10s} "
                f"| {row['pose']:^18s} | {row['stage']:^10d} | {row['desc']:<30s} |"
            )
            lines.append(line)
        
        # 4. Print once
        self.get_logger().info("\n" + "\n".join(lines))

        # 5. Increment counters for robots without updates
        for rid in self.no_update_counts:
            self.no_update_counts[rid] += 1
            # If a robot has gone stale, reset to NO_DATA
            if self.no_update_counts[rid] >= self.STALE_THRESHOLD and self.statuses[rid]['stage'] != 7:
                self.statuses[rid] = self.NO_DATA_ENTRY(rid)
                self.no_update_counts[rid] = 0

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
