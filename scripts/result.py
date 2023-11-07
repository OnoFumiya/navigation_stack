#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
import math
import tf.transformations as tf
from nav_msgs.msg import Odometry
import sys

start = False

pose_x = 0.0
pose_y = 0.0
def callback_odom(odom):
    global pose_x, pose_y
    pose_x = odom.pose.pose.position.x
    pose_y = odom.pose.pose.position.y

vel = Twist()
def callback_twist(vel_msg):
    global start, vel
    vel = vel_msg
    start = True


def main(file_name):
    global start, pose_x, pose_y, vel
    file_name_pass = str("/home/sobits/catkin_ws/src/" + file_name + ".csv")
    rospy.Subscriber("/odom", Odometry, callback_odom)
    rospy.Subscriber("/mobile_base/commands/velocity", Twist, callback_twist)
    r = rospy.Rate(10)
    f = open(file_name_pass, 'w')
    output_txt = "time" + "," + "vel_linear_value" + "," + "vel_angular" + "," + "pose_x" + "," + "pose_y" + "\n"
    f.write(output_txt)
    f.close()
    while not rospy.is_shutdown():
        if start:
            break
    init_time =  rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        f = open(file_name_pass, 'a')
        now_time =  rospy.Time.now().to_sec() - init_time
        vel_value = math.sqrt(vel.linear.x**2 + vel.linear.y**2)
        output_txt  = str(now_time) + ","
        output_txt += str(vel_value) + ","
        output_txt += str(vel.angular.z) + ","
        output_txt += str(pose_x) + ","
        output_txt += str(pose_y) + "\n"
        f.write(output_txt)
        f.close()
        r.sleep()

if __name__ == '__main__':
    rospy.init_node('result')
    args = sys.argv
    main(args[1])