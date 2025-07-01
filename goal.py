import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys
import time

class GoalPosePublisher(Node):
    def __init__(self, stop_time):
        super().__init__('goal_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # カウントダウン
        for i in range(stop_time):
            self.get_logger().info(f"start : {stop_time - i}")
            time.sleep(1)

        # 目標Pose作成・送信
        goal_msg = PoseStamped()
        now = self.get_clock().now().to_msg()
        goal_msg.header.stamp = now
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = 9.227359104156494
        goal_msg.pose.position.y = 0.16373726725578308
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = -0.012098718478021124
        goal_msg.pose.orientation.w = 0.9999268078270477

        self.publisher_.publish(goal_msg)
        self.get_logger().info("Published goal pose")

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run <your_package> goal_pose_publisher.py <wait_time_in_seconds>")
        sys.exit(1)

    stop_time = int(sys.argv[1])
    node = GoalPosePublisher(stop_time)
    rclpy.spin_once(node, timeout_sec=1.0)  # 少しだけスピンして確実に送信
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()