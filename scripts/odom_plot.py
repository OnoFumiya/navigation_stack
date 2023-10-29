#!/usr/bin/env python3
import rospy
# from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
import math
import tf.transformations as tf
from nav_msgs.msg import Odometry


pose_x = None
pose_y = None
yaw = None
def callback_odom(odom):
    global pose_x, pose_y, yaw
    pose_x = odom.pose.pose.position.x
    pose_y = odom.pose.pose.position.y
    # quat = [odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z]
    # rpy = tf.euler_from_quaternion(quat)
    # yaw = rpy[2]

    # sita = (2*(acos(odom.pose.pose.orientation.w + missed.orientation.w)))*((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w))/(std::fabs((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w)));
    if (math.fabs(odom.pose.pose.orientation.z * odom.pose.pose.orientation.w) == 0):
        yaw = 0.00
    else:
        yaw = 2 * math.acos(odom.pose.pose.orientation.w) * (odom.pose.pose.orientation.z * odom.pose.pose.orientation.w) / math.fabs(odom.pose.pose.orientation.z * odom.pose.pose.orientation.w)
        if (math.isnan(yaw)):
            yaw = 0.00
    # print(yaw)


def main():
    global pose_x, pose_y, yaw
    rospy.Subscriber('/odom', Odometry, callback_odom)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if ((pose_x != None) and (pose_y != None) and (yaw != None)):
            break
        r.sleep()
    while not rospy.is_shutdown():
        plt.cla()
        plt.arrow(pose_x, pose_y, 0.1 * math.cos(yaw), 0.1 * math.sin(yaw),head_length=0.02, head_width=0.02)
        plt.plot(pose_x, pose_y, "or")
        plt.xlim([-0.5, 1.5])
        plt.ylim([-1.0, 1.0])
        plt.pause(0.1)
        print("\npose_x = ",pose_x,"\npose_y = ",pose_y,"\nyaw = ",yaw, "\n")
        r.sleep()


if __name__ == '__main__':
    rospy.init_node('odom_plot')
    main()