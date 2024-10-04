#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion

def image_to_occupancy_grid(img_path):
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    height, width = img.shape

    grid = OccupancyGrid()
    grid.header = Header()
    grid.header.frame_id = "map"
    grid.info.resolution = 0.006  # Adjust as necessary
    grid.info.width = width
    grid.info.height = height
    grid.info.origin = Pose(Point(3.8776, 2.90286, 0), Quaternion(0, 0, 1, 0))

    # Convert image pixels to occupancy values (0 = free, 100 = occupied)
    occupancy_data = np.zeros((height, width), dtype=np.int8)
    occupancy_data[img > 127] = 0  # Free space
    occupancy_data[img <= 127] = 100  # Occupied space

    grid.data = occupancy_data.flatten().tolist()

    return grid

def publisher():
    rospy.init_node('image_to_grid')
    pub = rospy.Publisher('/base', OccupancyGrid, queue_size=10)

    rate = rospy.Rate(1)
    grid = image_to_occupancy_grid('/home/sobits/catkin_ws/src/navigation_stack/image/XY.jpg')

    while not rospy.is_shutdown():
        grid.header.stamp = rospy.Time.now()
        pub.publish(grid)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass