#include <stdio.h>
#include <ros/ros.h>
#include <dr_spaam_ros/LegPoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <limits>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Geometry>
#include <navigation_stack/WalkLegPoint.h>
#include <tf/transform_listener.h>


#include <algorithm>

#include <Eigen/Dense>
using namespace Eigen;

class ALL_PARAMETER
{
    public:
        float lidar_pose[2] = {0.2, 0.0};
        int sampling_step = 5;
        float max_human_vel = 1.5;
        float max_human_radius = 0.80;
        float human_noise = 0.4;
        float detect_range_time = 0.1;
        float velocity_weight_prediction = 2.0;
        float position_weight_prediction = 1.3333;
        float similar_object = 0.2;
        float no_moving_range = 0.6;
        std::string robot_base = "base_footprint";
        std::string map = "map";
        std::string removal_scan_topic = "/dr_spaam_navigation/scan";
        std::string stepping_point_topic = "/dr_spaam_navigation/object_pointers_step";
        std::string dr_spaam_topic = "/dr_spaam_detections";
        std::string merge_object_topic = "/object_point";
        std::string removal_scan_marker = "/dr_spaam_navigation/lidar_points";
        std::string step1_marker = "/dr_spaam_navigation/point_step1";
        std::string step2_marker = "/dr_spaam_navigation/point_step2";
        std::string step3_marker = "/dr_spaam_navigation/point_step3";
        std::string next_step_marker = "/dr_spaam_navigation/point_nextstep";
        std::string cost_topic = "/dr_spaam_navigation/point_cost";
        ALL_PARAMETER()
        {
            ros::NodeHandle nh;
            XmlRpc::XmlRpcValue param;
            nh.getParam("lidar_pose", param);
            lidar_pose[0] = static_cast<double>(param[0]);
            lidar_pose[1] = static_cast<double>(param[1]);
            nh.getParam("sampling_step", param);
            sampling_step = static_cast<int>(param);
            nh.getParam("max_human_vel", param);
            max_human_vel = static_cast<double>(param);
            nh.getParam("max_human_radius", param);
            max_human_radius = static_cast<double>(param);
            nh.getParam("human_noise", param);
            human_noise = static_cast<double>(param);
            nh.getParam("velocity_weight_prediction", param);
            velocity_weight_prediction = static_cast<double>(param);
            nh.getParam("position_weight_prediction", param);
            position_weight_prediction = static_cast<double>(param);
            nh.getParam("similar_object", param);
            similar_object = static_cast<double>(param);
            nh.getParam("no_moving_range", param);
            no_moving_range = static_cast<double>(param);
            nh.getParam("detect_range_time", param);
            detect_range_time = static_cast<double>(param);
            nh.getParam("robot_base", param);
            robot_base = static_cast<std::string>(param);
            nh.getParam("map", param);
            map = static_cast<std::string>(param);
            nh.getParam("removal_scan_topic", param);
            removal_scan_topic = static_cast<std::string>(param);
            nh.getParam("stepping_point_topic", param);
            stepping_point_topic = static_cast<std::string>(param);
            nh.getParam("dr_spaam_topic", param);
            dr_spaam_topic = static_cast<std::string>(param);
            nh.getParam("merge_object_topic", param);
            merge_object_topic = static_cast<std::string>(param);
            nh.getParam("removal_scan_marker", param);
            removal_scan_marker = static_cast<std::string>(param);
            nh.getParam("step1_marker", param);
            step1_marker = static_cast<std::string>(param);
            nh.getParam("step2_marker", param);
            step2_marker = static_cast<std::string>(param);
            nh.getParam("step3_marker", param);
            step3_marker = static_cast<std::string>(param);
            nh.getParam("next_step_marker", param);
            next_step_marker = static_cast<std::string>(param);
            nh.getParam("cost_topic", param);
            cost_topic = static_cast<std::string>(param);
        }
};


class MERGE_POINT
{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub_merge_object;
        ros::Subscriber leg_sub;
        ros::Subscriber scan_sub;
        ALL_PARAMETER all_parameter;
        dr_spaam_ros::LegPoseArray leg_point;
        dr_spaam_ros::LegPoseArray merge_data;
        std::vector<std::vector<geometry_msgs::Pose>> scan_points;
        std::vector<geometry_msgs::Pose> g_point;
        bool leg_call_ok = false;
        bool scan_call_ok = false;
        void callback_leg(const dr_spaam_ros::LegPoseArray &msg)
        {
            leg_point = msg;
            leg_call_ok = true;
        }
        void callback_scan(const sensor_msgs::LaserScan &msg)
        {
            scan_points.clear();
            g_point.clear();
            geometry_msgs::Pose pt;
            pt.position.z = 0.2;
            pt.orientation.w = 1.;
            pt.orientation.x = 0.;
            pt.orientation.y = 0.;
            pt.orientation.z = 0.;
            for (int i=0; i<msg.ranges.size(); i++)
            {
                if ((std::isnan(msg.ranges[i]) == false) && (msg.ranges[i] > 0.1))
                {
                    pt.position.x = msg.ranges[i]*(cos(msg.angle_min + msg.angle_increment*i));
                    pt.position.y = msg.ranges[i]*(sin(msg.angle_min + msg.angle_increment*i));
                    bool inputer = false;
                    for (int j=0; j<scan_points.size(); j++)
                    {
                        if (sqrtf(powf(scan_points[j][scan_points[j].size()-1].position.x - pt.position.x, 2) + powf(scan_points[j][scan_points[j].size()-1].position.y - pt.position.y, 2)) < all_parameter.similar_object)
                        {
                            g_point[j].position.x = (g_point[j].position.x*scan_points[j].size() + pt.position.x) / (scan_points[j].size()+1);
                            g_point[j].position.y = (g_point[j].position.y*scan_points[j].size() + pt.position.y) / (scan_points[j].size()+1);
                            scan_points[j].push_back(pt);
                            inputer = true;
                            break;
                        }
                    }
                    if (inputer != true)
                    {
                        std::vector<geometry_msgs::Pose> new_scan_point;
                        new_scan_point.push_back(pt);
                        scan_points.push_back(new_scan_point);
                        g_point.push_back(pt);
                    }
                }
            }
            for (int i=scan_points.size()-1; 0<=i; i--)
            {
                if (all_parameter.no_moving_range < sqrtf(powf(scan_points[i][scan_points[i].size()-1].position.x - scan_points[i][0].position.x, 2) + powf(scan_points[i][scan_points[i].size()-1].position.y - scan_points[i][0].position.y, 2)))
                {
                    g_point.erase(g_point.begin() + i);
                }
            }
            scan_call_ok = true;
        }
    public:
        MERGE_POINT()
        {
            leg_sub = nh.subscribe(all_parameter.dr_spaam_topic, 10, &MERGE_POINT::callback_leg, this);
            scan_sub = nh.subscribe("/scan", 10, &MERGE_POINT::callback_scan, this);
            pub_merge_object = nh.advertise<dr_spaam_ros::LegPoseArray>(all_parameter.merge_object_topic, 10);
            point_publisher();
        }
        void point_publisher()
        {
            while (ros::ok())
            {
                ros::spinOnce();
                if (leg_call_ok && scan_call_ok)
                {
                    break;
                }
            }
            ros::Rate loop_rate(10);
            while (ros::ok())
            {
                merge_data = leg_point;
                merge_data.poses.resize(g_point.size());
                copy(g_point.begin(), g_point.end(), merge_data.poses.begin());
                // std::vector<geometry_msgs::Pose> stack_point;
                for (int i=leg_point.poses.size()-1; i<=0; i++)
                {
                    if (i<0)
                    {
                        break;
                    }
                    for (int j=merge_data.poses.size()-1-i; 0<=j; j--)
                    {
                        if (j<0)
                        {
                            break;
                        }
                        if (sqrtf(powf(leg_point.poses[i].position.x - merge_data.poses[j].position.x, 2) + powf(leg_point.poses[i].position.y - merge_data.poses[j].position.y, 2)) <= all_parameter.human_noise)
                        {
                            merge_data.poses.erase(merge_data.poses.begin() + j);
                        }
                    }
                    merge_data.poses.push_back(leg_point.poses[i]);
                }
                // printf("\n");
                // for (int i=0; i<leg_point.poses.size(); i++)
                // {
                //     merge_data.poses.push_back(leg_point.poses[i]);
                // }
                if (merge_data.poses.size()==0)
                {
                    geometry_msgs::Pose no_pose;
                    no_pose.position.x = 100.0;
                    no_pose.position.y = 100.0;
                    no_pose.position.z = 0.;
                    no_pose.orientation.w = 1.;
                    no_pose.orientation.w = 0.;
                    no_pose.orientation.w = 0.;
                    no_pose.orientation.w = 0.;
                    merge_data.poses.push_back(no_pose);
                }
                pub_merge_object.publish(merge_data);
                ros::spinOnce();
                loop_rate.sleep();
            }
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_point_publisher");
    MERGE_POINT merge_point;
    ros::spin();
    return 0;
}