#!/usr/bin/env python3
import rospy
import math
import matplotlib.pyplot as plt
from nav_msgs.msg import Path


x_poses = None
y_poses = None
def callback_path(path):
    global x_poses, y_poses
    x_poses = []
    y_poses = []
    for i in range(len(path.poses)):
        x_poses += [path.poses[i].pose.position.x]
        y_poses += [path.poses[i].pose.position.y]


def main():
    global x_poses, y_poses
    # rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, callback_path)
    rospy.Subscriber('/move_base/DWAPlannerROS/global_plan', Path, callback_path)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if ((x_poses != None) and (y_poses != None)):
            break
        r.sleep()
    while not rospy.is_shutdown():
        plt.cla()
        plt.plot(x_poses[:], y_poses[:], "g-")
        plt.xlim([-0.5, 8.5])
        plt.ylim([-5.0, 5.0])
        plt.pause(0.1)
        r.sleep()


if __name__ == '__main__':
    rospy.init_node('gpp')
    main()