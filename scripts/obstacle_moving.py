#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np


current_x = None
current_y = None
center_dist = None
def callback_odom(msg):
    global current_x, current_y
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y


def callback_scan(msg):
    global center_dist
    center_dist = msg.ranges[len(msg.ranges)//2]


def main(mode, d, vel_x = 0.2):
    global current_x, current_y, center_dist
    rospy.init_node("obstacle_moving")
    vel = Twist()
    vel.linear.x = vel_x
    vel.linear.y = 0.0
    vel.angular.z = 0.0
    pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size= 10 )
    if (mode == "lidar"):
        sub = rospy.Subscriber("/scan", LaserScan, callback_scan)
        while not rospy.is_shutdown():
            if (center_dist is not None):
                break
    elif (mode == "odom"):
        sub = rospy.Subscriber("/odom", Odometry, callback_odom)
        while not rospy.is_shutdown():
            if (current_x is not None) and (current_y is not None):
                init_x = current_x
                init_y = current_y
                break
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        pub.publish(vel)
        r.sleep()
        if (mode == "lidar"):
            if (center_dist < d):
                break
        elif (mode == "odom"):
            if (d < np.sqrt((current_x - init_x)**2 + (current_y - init_y)**2)):
                break
    vel.linear.x = 0.0
    vel.linear.y = 0.0
    vel.angular.z = 0.0
    pub.publish(vel)



if __name__ == '__main__':
    main("lidar", 0.7, 0.2)
    # main("odom", 4.5, 0.2)
    rospy.spin()