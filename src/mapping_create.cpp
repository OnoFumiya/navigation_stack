#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <iostream>
#include <limits>
#include <math.h>
#include <typeinfo>
#include <sys/time.h>
#include <matplotlib-cpp/matplotlibcpp.h>
#include <navigation_stack/MapInformation.h>
#include <geometry_msgs/Pose.h>


// class GRIDDING
// {
//     public:
//         float size = 0.05;
// };

// class ROBOT_POSITION
// {   private:
//         ros::Subscriber sub_odom;
//         void callback_odom(const nav_msgs::Odometry &odom)
//         {
//             position_x = odom.pose.pose.position.x + misalignment.position.x;
//             position_y = odom.pose.pose.position.y + misalignment.position.y;
//             position_z = odom.pose.pose.position.z + misalignment.position.z;
//             sita = (2*(acos(odom.pose.pose.orientation.w + misalignment.orientation.w)))*((odom.pose.pose.orientation.z + misalignment.orientation.z)*(odom.pose.pose.orientation.w + misalignment.orientation.w))/(std::fabs((odom.pose.pose.orientation.z + misalignment.orientation.z)*(odom.pose.pose.orientation.w + misalignment.orientation.w)));
//             if ((std::fabs(sita)) > M_PI)
//             {
//                 sita = (2*M_PI - std::fabs(sita))*(((odom.pose.pose.orientation.z + misalignment.orientation.z)*(odom.pose.pose.orientation.w + misalignment.orientation.w))/(std::fabs((odom.pose.pose.orientation.z + misalignment.orientation.z)*(odom.pose.pose.orientation.w + misalignment.orientation.w))));
//             }
//         }
//     public:
//         geometry_msgs::Pose misalignment;
//         float position_x = 100.0, position_y = 100.0, position_z = 0.0, sita;
//         ROBOT_POSITION()
//         {
//             ros::NodeHandle node;
//             sub_odom = node.subscribe("/odom", 10, &ROBOT_POSITION::callback_odom, this);
//             get_point();
//         }
//         void get_point()
//         {
//             ros::spinOnce();
//             while (1)
//             {
//                 misalignment.position.x = 0.0;
//                 misalignment.position.y = 0.0;
//                 misalignment.position.z = 0.0;
//                 misalignment.orientation.x = 0.0;
//                 misalignment.orientation.y = 0.0;
//                 misalignment.orientation.z = 0.0;
//                 misalignment.orientation.w = 0.0;
//                 ros::spinOnce();
//                 if ((position_x != 100) || (position_y != 100))
//                 {
//                     break;
//                 }
//             }
//         }
// };


// class OBSTACLE_DIST
// {
//     private:
//         ros::Subscriber sub_dist;
//         ROBOT_POSITION robot_position;
//         float maxdist, mindist;
//         int i;
//         void callback_obstacle(const sensor_msgs::LaserScan &ob)
//         {
//             range.clear();
//             range_point.clear();
//             float a;
//             range_angle_increment = ob.angle_increment;
//             maxdist = ob.range_max;
//             mindist = ob.range_min;
//             for (i=0; i<ob.ranges.size(); i++)
//             {
//                 if ((mindist <= ob.ranges[i]) && (ob.ranges[i] <= maxdist))
//                 {
//                     a = (robot_position.sita + ob.angle_min + range_angle_increment*i);
//                     range.push_back(ob.ranges[i]);
//                     range_point.push_back({robot_position.position_x + (ob.ranges[i]*(cos(a))), robot_position.position_y + (ob.ranges[i]*(sin(a))), 0, a});
//                 }
//             }
//             range_size = range.size();
//         }
//     public:
//         std::vector<float> range;
//         std::vector<std::vector<float>> range_point;
//         long range_size = std::numeric_limits<int>::max();
//         float range_angle_increment/*, range_max, range_min*/;
//         OBSTACLE_DIST()
//         {
//             ros::NodeHandle node;
//             sub_dist = node.subscribe("/scan", 10, &OBSTACLE_DIST::callback_obstacle, this);
//             get_dist();
//         }
//         void get_dist()
//         {
//             ros::spinOnce();
//             // range_min = std::numeric_limits<int>::max();
//             // range_max = (std::numeric_limits<int>::max())*(-1);
//             while(1)
//             {
//                 ros::spinOnce();
//                 if (range_size == range.size())
//                 {
//                     ros::spinOnce();
//                     break;
//                 }
//             }
//             ros::spinOnce();
//             // while(1)
//             // {
//             //     ros::spinOnce();
//             //     if (range_point.size() == range_size)
//             //     {
//             //         ros::spinOnce();
//             //         break;
//             //     }
//             // }
//             // ros::spinOnce();
//             // for (int j=range.size()-1;j>=0;j--)
//             // {
//             //     if (((range[j]<0)||(maxdist<range[j])))
//             //     {
//             //         range.erase(std::cbegin(range) + j);
//             //         range_point.erase(std::cbegin(range_point) + j);
//             //     }
//             // }
//             // ros::spinOnce();
//             // for (int j=0; j<range.size(); j++)
//             // {
//             //     if (range_max < range[j])
//             //     {
//             //         range_max = range[j];
//             //     }
//             //     if (range_min > range[j])
//             //     {
//             //         range_min = range[j];
//             //     }
//             // }
//         }
// };

// class MAP_CREATE
// {
//     private:
//         ros::Publisher pub_global_map;
//         ros::Subscriber sub_local_map;
//         ros::Subscriber sub_misalignment;
//         // ros::Publisher pub_local_map;
//         // ros::Subscriber sub_start;
//         // void callback_mapping_start(const std_msgs::Bool &start)
//         // {
//         //     start_judge.data = start.data;
//         // }
//         void callback_misalignment(const geometry_msgs::Pose &miss)
//         {
//             robot_position.misalignment.position.x = miss.position.x;
//             robot_position.misalignment.position.y = miss.position.y;
//             robot_position.misalignment.position.z = miss.position.z;
//             robot_position.misalignment.orientation.x = miss.orientation.x;
//             robot_position.misalignment.orientation.y = miss.orientation.y;
//             robot_position.misalignment.orientation.z = miss.orientation.z;
//             robot_position.misalignment.orientation.w = miss.orientation.w;
//             create_frag = true;
//         }
//     public:
//         ROBOT_POSITION robot_position;
//         navigation_stack::MapInformation map;
//         bool create_frag = false;
//         MAP_CREATE()
//         {
//             ros::NodeHandle node;
//             pub_global_map = node.advertise<navigation_stack::MapInformation>("/global_map", 10);
//             sub_misalignment = node.subscribe("/localization_missed", 10, &MAP_CREATE::callback_misalignment, this);
//             // sub_start = node.subscribe("/mapping_start", 10, &MAP_CREATE::callback_mapping_start, this);
//             // pub_local_map = node.advertise<navigation_stack::MapInformation>("/local_map", 10);
//             before_create();
//             create_map();
//         }
//         void before_create()
//         {
//             while (ros::ok())
//             {
//                 if (create_frag)
//                 {
//                     ros::spinOnce();
//                     break;
//                 }
//                 ros::spinOnce();
//             }
//         }
//         // geometry_msgs::Pose misalignment;
//         void create_map()
//         {
//             OBSTACLE_DIST obstacle_dist;
//             GRIDDING gridding;
//             bool pushback_ok_frag;
//             geometry_msgs::Vector3 vec;
//             vec.z = 0;
//             ros::spinOnce();
//             while (ros::ok())
//             {
//                 for (int i=0; i<obstacle_dist.range.size(); i++)
//                 {
//                     vec.x = ss(obstacle_dist.range_point[i][0],gridding.size);
//                     vec.y = ss(obstacle_dist.range_point[i][1],gridding.size);
//                     pushback_ok_frag = true;
//                     for (int j=0; j<map.cost.size(); j++)
//                     {
//                         if ((((map.cost[j].x-(gridding.size/2)) <= vec.x) && (vec.x < (map.cost[j].x+(gridding.size/2)))) && (((map.cost[j].y-(gridding.size/2)) <= vec.y) && (vec.y < (map.cost[j].y+(gridding.size/2)))))
//                         {
//                             pushback_ok_frag = false;
//                             break;
//                         }
//                     }
//                     if (pushback_ok_frag)
//                     {
//                         map.cost.push_back(vec);
//                         printf("aaa\n");
//                     }
//                     ros::spinOnce();
//                 }
//                 pub_global_map.publish(map);
//                 ros::spinOnce();
//             }
//         }
//         float ss(float s, float g, bool f=true)
//         {
//             float r = s - (((float)(s/g) - (int)(s/g))*g);
//             if ((s<0) && (f))
//             {
//                 r-=g;
//             }
//             r += (g/2);
//             return r;
//         }
// };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_create");
    // MAP_CREATE map_create;
    ros::spin();
}
