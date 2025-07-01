import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys
import time
from ament_index_python.packages import get_package_share_directory

class ResultLogger(Node):
    def __init__(self, file_name):
        super().__init__('result_logger')

        # 状態変数
        self.start = False
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.vel = Twist()
        self.start_time = None

        # ファイルパス生成
        self.file_path = f"/home/sobits/colcon_ws/src/{file_name}.csv"

        # ファイル初期化
        with open(self.file_path, 'w') as f:
            f.write("time,vel_linear_value,vel_angular,pose_x,pose_y\n")

        # サブスクライバ
        self.create_subscription(Odometry, '/sobit_pro/odom', self.odom_callback, 1)
        self.create_subscription(Twist, '/sobit_pro/cmd_vel', self.twist_callback, 1)

        # タイマーループ（10Hz）
        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y

    def twist_callback(self, msg):
        self.vel = msg
        if not self.start:
            self.start = True
            self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
            self.get_logger().info("Start recording...")

    def timer_callback(self):
        if not self.start:
            return

        now_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        vel_value = math.hypot(self.vel.linear.x, self.vel.linear.y)
        output_txt = f"{now_time},{vel_value},{self.vel.angular.z},{self.pose_x},{self.pose_y}\n"

        with open(self.file_path, 'a') as f:
            f.write(output_txt)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run <your_package> result_logger.py <file_name>")
        sys.exit(1)

    file_name = sys.argv[1]
    node = ResultLogger(file_name)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
