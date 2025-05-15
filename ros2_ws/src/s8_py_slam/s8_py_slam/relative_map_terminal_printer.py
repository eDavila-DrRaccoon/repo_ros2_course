import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from datetime import datetime
try:
    from zoneinfo import ZoneInfo  # Available in Python 3.9+ (For Foxy/Ubuntu 22.04 with Python 3.10)
except ImportError:
    from backports.zoneinfo import ZoneInfo  # For Python < 3.9 (For Humble/Ubuntu 20.04 with Python 3.8)
import math

# ANSI color codes
GREEN = '\033[32m'
RED = '\033[31m'
BLUE = '\033[34m'
WHITE = '\033[37m'
RESET = '\033[0m'

class MapTerminalPrinter(Node):
    def __init__(self):
        super().__init__('relative_map_terminal_printer')
        # Map subscription
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        # Pose subscription
        self.create_subscription(
            PoseStamped,
            '/pose_stamped',
            self.pose_callback,
            10)

        # Map & robot state
        self.map_data = None
        self.map_width = self.map_height = 0
        self.map_origin_x = self.map_origin_y = 0.0
        self.robot_x = None
        self.robot_y = None
        self.robot_theta = None

        # Visualization params
        self.resolution = 0.05  # m per cell
        self.window_size = 41   # chars per side
        self.half = self.window_size // 2

        # Display characters
        self.origin_char = '+'
        self.free_char = '·'
        self.occ_char = '#'
        self.unk_char = '?'

        # Refresh timer
        self.create_timer(0.033, self.print_window)

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    def pose_callback(self, msg: PoseStamped):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny, cosy)

    def print_window(self):
        if self.map_data is None or self.robot_x is None:
            # self.get_logger().info('Waiting for map and pose...')
            return

        # prepare empty grid
        grid = [[ ' ' for _ in range(self.window_size)]
                for _ in range(self.window_size)]

        # sample map around robot center
        for row in range(self.window_size):
            for col in range(self.window_size):
                # world coords relative to robot
                wx = self.robot_x + (col - self.half) * self.resolution
                wy = self.robot_y + (self.half - row) * self.resolution
                # convert to map indices
                mx = int((wx - self.map_origin_x) / self.resolution)
                my = int((wy - self.map_origin_y) / self.resolution)
                if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                    val = self.map_data[my * self.map_width + mx]
                    if abs(wx) < self.resolution/2 and abs(wy) < self.resolution/2:
                        # mark global origin relative to view
                        grid[row][col] = f'{GREEN}{self.origin_char}{RESET}'
                    elif val == -1:
                        grid[row][col] = self.unk_char
                    elif val == 0:
                        grid[row][col] = self.free_char
                    else:
                        grid[row][col] = self.occ_char

        # overlay robot arrows at center
        center = self.half
        arrows = [['↖','↑','↗'], ['←','R','→'], ['↙','↓','↘']]
        dir_map = [(1,2),(0,2),(0,1),(0,0),(1,0),(2,0),(2,1),(2,2)]
        angle = (self.robot_theta % (2*math.pi) + 2*math.pi) % (2*math.pi)
        sector = int((angle + math.pi/8) // (math.pi/4)) % 8
        for i in range(3):
            for j in range(3):
                r = center + (i-1)
                c = center + (j-1)
                ch = arrows[i][j]
                if (i,j) == (1,1):
                    grid[r][c] = f'{BLUE}{ch}{RESET}'
                elif (i,j) == dir_map[sector]:
                    grid[r][c] = f'{RED}{ch}{RESET}'
                else:
                    grid[r][c] = f'{WHITE}{ch}{RESET}'

        # clear screen
        print("\033[2J\033[H", end='')

        # Record the local “snapshot” time
        now_local = datetime.now(ZoneInfo("America/Mexico_City"))
        snap_str = now_local.strftime('%Y-%m-%d %H:%M:%S.%f')[:-4]
        header = f"--- Map snapshot at local time: {snap_str} ---"
        print(f"{header}\n")

        # Print the map
        for row in grid:
            print(''.join(row))

def main():
    rclpy.init()
    node = MapTerminalPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
