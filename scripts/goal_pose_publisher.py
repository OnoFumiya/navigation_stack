#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import sys

def main(stop_time):
    pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size= 10 )
    for i in range(stop_time):
        print("start : ", (stop_time-i))
        rospy.sleep(1)
    nav_goal = PoseStamped()
    nav_goal.header.stamp = rospy.Time.now()
    nav_goal.header.frame_id = "map"
    nav_goal.pose.position.x = 4.227359104156494
    nav_goal.pose.position.y = 0.16373726725578308
    nav_goal.pose.position.z = 0.0
    nav_goal.pose.orientation.x = 0.0
    nav_goal.pose.orientation.y = 0.0
    nav_goal.pose.orientation.z = -0.012098718478021124
    nav_goal.pose.orientation.w = 0.9999268078270477
    pub_goal.publish(nav_goal)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("goal_pose_publisher")
    args = sys.argv
    main((int)(args[1]))