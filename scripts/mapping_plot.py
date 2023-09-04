#!/usr/bin/env python3
import rospy
# from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
# from matplotlib import patches
import math
from navigation_stack.msg import MapInformation
from geometry_msgs.msg import Vector3
# from geometry_msgs.msg import Pose
# geometry_msgs/Pose misalignment
from nav_msgs.msg import Odometry
#include <nav_msgs/Odometry.h>

# class GRIDDING():
#     size = 0.05
#     cost_size = 0.2

posi_x = 0
posi_y = 0
nx = []
ny = []
def callback_odom(odom):
    global posi_x, posi_y
    posi_x = odom.pose.pose.position.x
    posi_y = odom.pose.pose.position.y

def callback_node(node):
    global nx, ny
    nx = []
    ny = []
    for i in range(len(node.cost)):
        nx = nx + [(-1)*(node.cost[i]).y]
        nx = nx + [     (node.cost[i]).x]

def callback_map(map):
    global posi_x, posi_y, nx, ny
    plt.cla()
    ax = plt.gca()
    ax.set_facecolor('gray')
    limxmin = float('inf')
    limxmax = (float('inf'))*(-1)
    limymin = float('inf')
    limymax = (float('inf'))*(-1)
    cost_x = []
    cost_y = []
    clearly_x = []
    clearly_y = []
    for i in range(len(map.cost)):
        cost_x = cost_x + [(-1)*(map.cost[i]).y]
        cost_y = cost_y + [     (map.cost[i]).x]
        # if ((-1)*(map.cost[i]).y < limxmin):
        #     limxmin = (-1)*(map.cost[i]).y
        # if (limxmax < (-1)*(map.cost[i]).y):
        #     limxmax = (-1)*(map.cost[i]).y
        # if ((map.cost[i]).x < limymin):
        #     limymin = (map.cost[i]).x
        # if (limymax < (map.cost[i]).x):
        #     limymax = (map.cost[i]).x
    for i in range(len(map.clearly)):
        clearly_x = clearly_x + [(-1)*(map.clearly[i]).y]
        clearly_y = clearly_y + [     (map.clearly[i]).x]
    # plt.xlim([limxmin - 0.5, limxmax + 0.5])
    # plt.ylim([limymin - 0.5, limymax + 0.5])
    plt.plot(clearly_x, clearly_y, 'sw')
    plt.plot(cost_x, cost_y, 'sk')
    # plt.scatter(clearly_x, clearly_y, linestyle='None', color='y', marker='s', s=30)
    # plt.scatter(cost_x, cost_y, linestyle='None', color='k', marker='s', s=30)
    # plt.plot(-0.3,0.3,'og')
    plt.plot(posi_y*(-1), posi_x ,'or')
    plt.pause(0.001)


def main():
    rospy.Subscriber('/odom', Odometry, callback_odom)
    # rospy.Subscriber('/astar_node', MapInformation, callback_node)
    rospy.Subscriber('/global_map', MapInformation, callback_map)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('mapping_plot')
    main()