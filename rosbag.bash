#!/bin/bash

rosbag_file="rosbag_file"

if [ $# = 0 ]; then
    echo "rosbag file name = " $rosbag_file
else
    echo "rosbag file name = " $1
    rosbag_file=$1
fi

cd ~/catkin_ws/src/navigation_developer
rosbag record   /dr_spaam_detections  \
                /navigation_developer/scan  \
                /odom  \
                /scan  \
                /tf  \
                /tf_static  \
                /urg_node/parameter_descriptions  \
                /urg_node/parameter_updates  \
                -o $rosbag_file
