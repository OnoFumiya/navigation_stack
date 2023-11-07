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

// using namespace std;

class ALL_PARAMETER
{
    // private:
    //     ros::NodeHandle nh;
    //     XmlRpc::XmlRpcValue param;
    public:
        float lidar_pose[2] = {0.2, 0.0};
        int sampling_step = 5;
        float max_human_vel = 1.5;
        float max_human_radius = 0.80;
        float human_noise = 0.4;
        float detect_range_time = 0.1;
        float x_weght_prediction = 2.0;
        float y_weght_prediction = 1.3333;
        std::string robot_base = "base_footprint";
        std::string map = "map";
        std::string removal_scan_topic = "/dr_spaam_navigation/scan";
        std::string stepping_leg_point_topic = "/dr_spaam_navigation/leg_pointers_step";
        std::string dr_spaam_topic = "/dr_spaam_detections";
        std::string removal_scan_marker = "/dr_spaam_navigation/lidar_points";
        std::string step1_leg_marker = "/dr_spaam_navigation/leg_point_step1";
        std::string step2_leg_marker = "/dr_spaam_navigation/leg_point_step2";
        std::string step3_leg_marker = "/dr_spaam_navigation/leg_point_step3";
        std::string next_step_leg_marker = "/dr_spaam_navigation/leg_point_nextstep";
        std::string walk_cost_topic = "/dr_spaam_navigation/leg_cost";
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
            nh.getParam("x_weght_prediction", param);
            x_weght_prediction = static_cast<double>(param);
            nh.getParam("y_weght_prediction", param);
            y_weght_prediction = static_cast<double>(param);
            nh.getParam("detect_range_time", param);
            detect_range_time = static_cast<double>(param);
            nh.getParam("robot_base", param);
            robot_base = static_cast<std::string>(param);
            nh.getParam("map", param);
            map = static_cast<std::string>(param);
            nh.getParam("removal_scan_topic", param);
            removal_scan_topic = static_cast<std::string>(param);
            nh.getParam("stepping_leg_point_topic", param);
            stepping_leg_point_topic = static_cast<std::string>(param);
            nh.getParam("dr_spaam_topic", param);
            dr_spaam_topic = static_cast<std::string>(param);
            nh.getParam("removal_scan_marker", param);
            removal_scan_marker = static_cast<std::string>(param);
            nh.getParam("step1_leg_marker", param);
            step1_leg_marker = static_cast<std::string>(param);
            nh.getParam("step2_leg_marker", param);
            step2_leg_marker = static_cast<std::string>(param);
            nh.getParam("step3_leg_marker", param);
            step3_leg_marker = static_cast<std::string>(param);
            nh.getParam("next_step_leg_marker", param);
            next_step_leg_marker = static_cast<std::string>(param);
            nh.getParam("walk_cost_topic", param);
            walk_cost_topic = static_cast<std::string>(param);
        }
};


class HUMAN_DETECT
{
    private:
        ros::NodeHandle nh;
        tf::TransformListener tf_listener;
        ros::Publisher pub_walk_leg;
        ros::Publisher pub_legremoval_scan;
        ros::Subscriber leg_sub;
        ALL_PARAMETER all_parameter;
        // float lidar_pose[2] = {0.2, 0.0};
        // int sampling_step = 5;
        // float max_human_vel = 1.5;
        // float max_human_radius = 0.80;
        // float human_noise = 0.4;
        // float detect_range_time = 0.1;
        bool start_flag;
        std::vector<geometry_msgs::Point> leg_points;
        std::vector<geometry_msgs::Point> leg_points_base;
        geometry_msgs::Point Pointtransform(const std::string& org_frame, const std::string& target_frame, const geometry_msgs::Point& point)
        {
            geometry_msgs::PointStamped ptstanp_transformed;
            geometry_msgs::PointStamped ptstanp;
            geometry_msgs::Point pt_transformed;
            ptstanp.header.frame_id = org_frame;
            ptstanp.header.stamp = ros::Time(0);
            ptstanp.point = point;
            try
            {
                tf_listener.transformPoint( target_frame, ptstanp, ptstanp_transformed );
                pt_transformed.x = ptstanp_transformed.point.x;
                pt_transformed.y = ptstanp_transformed.point.y;
                pt_transformed.z = ptstanp_transformed.point.z;
            }
            catch( const tf::TransformException& ex )
            {
                ROS_ERROR( "%s",ex.what( ) );
                pt_transformed.x = NAN;
                pt_transformed.y = NAN;
                pt_transformed.z = NAN;
            }
            return pt_transformed;
        }
        void callback_leg(const dr_spaam_ros::LegPoseArray &msg)
        {
            leg_points.clear();
            leg_points_base.clear();
            geometry_msgs::Point point;
            for (int i=0; i<msg.poses.size(); i++)
            {
                point.x = msg.poses[i].position.x + all_parameter.lidar_pose[0];
                point.y = msg.poses[i].position.y + all_parameter.lidar_pose[1];
                point.z = 0.2;
                // if ((msg.scan.range_min <= sqrtf(powf(point.x, 2.) + powf(point.y, 2.))) && (sqrtf(powf(point.x, 2.) + powf(point.y, 2.)) <= msg.scan.range_max))
                if ((0.25 <= sqrtf(powf(point.x, 2.) + powf(point.y, 2.))) && (sqrtf(powf(point.x, 2.) + powf(point.y, 2.)) <= msg.scan.range_max))
                {
                    leg_points_base.push_back(point);
                    leg_points.push_back(Pointtransform(all_parameter.robot_base, all_parameter.map, point));
                }
            }

            sensor_msgs::LaserScan removal_lidar_data;
            removal_lidar_data.header.seq = msg.scan.header.seq;
            removal_lidar_data.header.stamp = msg.scan.header.stamp;
            removal_lidar_data.header.frame_id = msg.scan.header.frame_id;
            removal_lidar_data.angle_min = msg.scan.angle_min;
            removal_lidar_data.angle_max = msg.scan.angle_max;
            removal_lidar_data.angle_increment = msg.scan.angle_increment;
            removal_lidar_data.time_increment = msg.scan.time_increment;
            removal_lidar_data.scan_time = msg.scan.scan_time;
            removal_lidar_data.range_min = msg.scan.range_min;
            removal_lidar_data.range_max = msg.scan.range_max;
            removal_lidar_data.ranges.resize(msg.scan.ranges.size());
            copy(msg.scan.ranges.begin(), msg.scan.ranges.end(), removal_lidar_data.ranges.begin());
            float range_angle_increment = msg.scan.angle_increment;
            for (int i=0; i<msg.scan.ranges.size(); i++)
            {
                if ((msg.scan.range_min <= msg.scan.ranges[i]) && (msg.scan.ranges[i] <= msg.scan.range_max))
                {
                    for (int j=0; j<leg_points_base.size(); j++)
                    {
                        if (sqrtf(powf((msg.scan.ranges[i]*(cos(msg.scan.angle_min + range_angle_increment*i)) + all_parameter.lidar_pose[0]) - leg_points_base[j].x, 2.) + powf((msg.scan.ranges[i]*(sin(msg.scan.angle_min + range_angle_increment*i)) + all_parameter.lidar_pose[1]) - leg_points_base[j].y, 2.)) <= all_parameter.max_human_radius)
                        {
                            removal_lidar_data.ranges[i] = NAN;
                            break;
                        }
                    }
                }
            }
            pub_legremoval_scan.publish(removal_lidar_data);
            start_flag = true;
        }
    public:
        HUMAN_DETECT()
        {
            pub_walk_leg = nh.advertise<navigation_stack::WalkLegPoint>(all_parameter.stepping_leg_point_topic, 10);
            pub_legremoval_scan = nh.advertise<sensor_msgs::LaserScan>(all_parameter.removal_scan_topic, 10);
            leg_sub = nh.subscribe(all_parameter.dr_spaam_topic, 10, &HUMAN_DETECT::callback_leg, this);
            start_flag = false;
            while (ros::ok())
            {
                ros::spinOnce();
                if (start_flag)
                {
                    break;
                }
            }
            publisher();
        }
        void publisher()
        {
            navigation_stack::WalkLegPoint leg_points_steps;
            leg_points_steps.point1.resize(leg_points.size());
            leg_points_steps.point2.resize(leg_points.size());
            leg_points_steps.point3.resize(leg_points.size());
            copy(leg_points.begin(), leg_points.end(), leg_points_steps.point1.begin());
            copy(leg_points.begin(), leg_points.end(), leg_points_steps.point2.begin());
            copy(leg_points.begin(), leg_points.end(), leg_points_steps.point3.begin());
            leg_points_steps.point_next.clear();
            while (ros::ok())
            {
                leg_points_steps.point1.clear();
                leg_points_steps.point1.resize(leg_points_steps.point2.size());
                copy(leg_points_steps.point2.begin(), leg_points_steps.point2.end(), leg_points_steps.point1.begin());
                leg_points_steps.point2.clear();
                leg_points_steps.point2.resize(leg_points_steps.point3.size());
                copy(leg_points_steps.point3.begin(), leg_points_steps.point3.end(), leg_points_steps.point2.begin());

                std::vector<std::vector<geometry_msgs::Point>> leg_points_samplings;   // 第一要素が足の数、第二要素が足のそれぞれのステップ
                std::vector<geometry_msgs::Point> leg_points_temp;
                leg_points_samplings.clear();
                leg_points_samplings.resize(leg_points.size());
                leg_points_temp.resize(leg_points.size());
                copy(leg_points.begin(), leg_points.end(), leg_points_temp.begin());
                for (int i=0; i<leg_points_temp.size(); i++)
                {
                    leg_points_samplings[i].push_back(leg_points_temp[i]);
                }
                for (int i=1; i<all_parameter.sampling_step; i++)
                {
                    ros::Duration(all_parameter.detect_range_time).sleep();
                    leg_points_temp.resize(leg_points.size());
                    copy(leg_points.begin(), leg_points.end(), leg_points_temp.begin());
                    for (int j=0; j<leg_points_temp.size(); j++)
                    {
                        bool input_flag = false;
                        for (int k=0; k<leg_points_samplings.size(); k++)
                        {
                            if (sqrtf(powf(leg_points_temp[j].x - leg_points_samplings[k][leg_points_samplings[k].size()-1].x, 2.) + powf(leg_points_temp[j].y - leg_points_samplings[k][leg_points_samplings[k].size()-1].y, 2.)) <= all_parameter.human_noise)
                            {
                                leg_points_samplings[k].push_back(leg_points_temp[j]);
                                input_flag = true;
                            }
                        }
                        if (input_flag != true)
                        {
                            std::vector<geometry_msgs::Point> new_input_leg;
                            new_input_leg.clear();
                            new_input_leg.push_back(leg_points_temp[j]);
                            leg_points_samplings.push_back(new_input_leg);
                        }
                    }
                }

                geometry_msgs::Point leg_point_temp;
                geometry_msgs::Point pt_nan;
                pt_nan.x = NAN;
                pt_nan.y = NAN;
                pt_nan.z = 0.0;
                leg_points_steps.point3.clear();
                leg_points_steps.point3.resize(leg_points_steps.point2.size(), pt_nan);
                for (int i=0; i<leg_points_samplings.size(); i++)
                {
                    float x = 0.0;
                    float y = 0.0;
                    for (int j=0; j<leg_points_samplings[i].size(); j++)
                    {
                        x += leg_points_samplings[i][j].x;
                        y += leg_points_samplings[i][j].y;
                    }
                    leg_point_temp.x = x/leg_points_samplings[i].size();
                    leg_point_temp.y = y/leg_points_samplings[i].size();
                    leg_point_temp.z = 0.2;
                    bool input_flag = false;
                    for (int j=0; j<leg_points_steps.point2.size(); j++)
                    {
                        if (sqrtf(powf(leg_point_temp.x - leg_points_steps.point2[j].x, 2.) + powf(leg_point_temp.y - leg_points_steps.point2[j].y, 2.)) <= all_parameter.max_human_vel*all_parameter.detect_range_time*all_parameter.sampling_step)
                        {
                            leg_points_steps.point3[j].x = leg_point_temp.x;
                            leg_points_steps.point3[j].y = leg_point_temp.y;
                            leg_points_steps.point3[j].z = leg_point_temp.z;
                            input_flag = true;
                            break;
                        }
                    }
                    if (input_flag != true)
                    {
                        leg_points_steps.point3.push_back(leg_point_temp);
                    }
                }

                for (int i=0; i<leg_points_steps.point3.size(); i++)
                {
                    if ((std::isnan(leg_points_steps.point3[i].x)) || (std::isnan(leg_points_steps.point3[i].y)))
                    {
                        leg_points_steps.point3.erase(leg_points_steps.point3.begin() + i);
                        if ((0 <= i) && (i < leg_points_steps.point1.size()))
                        {
                            leg_points_steps.point1.erase(leg_points_steps.point1.begin() + i);
                        }
                        if ((0 <= i) && (i < leg_points_steps.point2.size()))
                        {
                            leg_points_steps.point2.erase(leg_points_steps.point2.begin() + i);
                        }
                    }
                }

                // ここから予測
                geometry_msgs::Point p1, p2, p3;
                // float a_base_x, b_base_x, c_base_x, a_base_y, b_base_y, c_base_y;
                // a_base_x = NAN;
                // b_base_x = NAN;
                // c_base_x = NAN;
                // a_base_y = NAN;
                // b_base_y = NAN;
                // c_base_y = NAN;
                int mode = 0;   // mode : 0(1_arrow), 1(x), 2(y)
                float a, b, c;
                leg_points_steps.point_next.clear();
                for (int i=0; i<leg_points_steps.point3.size(); i++)
                {
                    if ((i < leg_points_steps.point1.size()) && (i < leg_points_steps.point2.size()))
                    {
                        if ((std::isnan(leg_points_steps.point1[i].x) != true) && (std::isnan(leg_points_steps.point1[i].y) != true) && (std::isnan(leg_points_steps.point2[i].x) != true) && (std::isnan(leg_points_steps.point2[i].y) != true) && (std::isnan(leg_points_steps.point3[i].x) != true) && (std::isnan(leg_points_steps.point3[i].y) != true))
                        {
                            p1 = Pointtransform(all_parameter.map, all_parameter.robot_base, leg_points_steps.point1[i]);
                            p2 = Pointtransform(all_parameter.map, all_parameter.robot_base, leg_points_steps.point2[i]);
                            p3 = Pointtransform(all_parameter.map, all_parameter.robot_base, leg_points_steps.point3[i]);
                            MatrixXd A(3, 2);
                            VectorXd B(3);
                            A(0, 0) = p1.x;
                            A(0, 1) = 1.0;
                            B(0) = p1.y;
                            A(1, 0) = p2.x;
                            A(1, 1) = 1.0;
                            B(1) = p2.y;
                            A(2, 0) = p3.x;
                            A(2, 1) = 1.0;
                            B(2) = p3.y;
                            Vector2d result = A.colPivHouseholderQr().solve(B);
                            float d1 = std::fabs(result[0]*p1.x - 1*p1.y + result[1]) / sqrtf(powf(result[0], 2.) + 1.);
                            float d2 = std::fabs(result[0]*p2.x - 1*p2.y + result[1]) / sqrtf(powf(result[0], 2.) + 1.);
                            float d3 = std::fabs(result[0]*p3.x - 1*p3.y + result[1]) / sqrtf(powf(result[0], 2.) + 1.);
                            if ((d1 <= all_parameter.human_noise/2.) && (d2 <= all_parameter.human_noise/2.) && (d3 <= all_parameter.human_noise/2.))
                            {
                                mode = 0;
                            }
                            else if ((p1.x != p2.x) && (p2.x != p3.x) && (p3.x != p1.x) && ((p1.y != p2.y) && (p2.y != p3.y) && (p3.y != p1.y)))
                            {
                                if ((((p1.x < p2.x) && (p2.x < p3.x)) || ((p1.x > p2.x) && (p2.x > p3.x))) && (((p1.y < p2.y) && (p2.y < p3.y)) || ((p1.y > p2.y) && (p2.y > p3.y))))
                                {
                                    float a_base_x, b_base_x, c_base_x, a_base_y, b_base_y, c_base_y;
                                    a_base_x = ((p1.y - p2.y) / ((p1.x - p2.x) * (p2.x - p3.x))) - ((p1.y - p3.y) / ((p1.x - p3.x) * (p2.x - p3.x)));
                                    b_base_x = (p1.y - p2.y) / (p1.x - p2.x) - a_base_x * (p1.x + p2.x);
                                    c_base_x = p1.y - a_base_x * powf(p1.x, 2.) - b_base_x * p1.x;
                                    a_base_y = ((p1.x - p2.x) / ((p1.y - p2.y) * (p2.y - p3.y))) - ((p1.x - p3.x) / ((p1.y - p3.y) * (p2.y - p3.y)));
                                    b_base_y = (p1.x - p2.x) / (p1.y - p2.y) - a_base_y * (p1.y + p2.y);
                                    c_base_y = p1.x - a_base_y * powf(p1.y, 2.) - b_base_y * p1.y;
                                    if ((((p1.x < p2.x) && (p2.x < p3.x) && (p3.x < ((-1)*(b_base_x / (2*a_base_x))))) || ((p1.x > p2.x) && (p2.x > p3.x) && (p3.x > ((-1)*(b_base_x / (2*a_base_x)))))) && 
                                        (((p1.y < p2.y) && (p2.y < p3.y) && (p3.y < ((-1)*(b_base_y / (2*a_base_y))))) || ((p1.y > p2.y) && (p2.y > p3.y) && (p3.y > ((-1)*(b_base_y / (2*a_base_y)))))))
                                    {
                                        mode = 0;
                                    }
                                    else if (((p1.x < p2.x) && (p2.x < p3.x) && (p3.x < ((-1)*(b_base_x / (2*a_base_x))))) || ((p1.x > p2.x) && (p2.x > p3.x) && (p3.x > ((-1)*(b_base_x / (2*a_base_x))))))
                                    {
                                        mode = 1;
                                    }
                                    else if (((p1.y < p2.y) && (p2.y < p3.y) && (p3.y < ((-1)*(b_base_y / (2*a_base_y))))) || ((p1.y > p2.y) && (p2.y > p3.y) && (p3.y > ((-1)*(b_base_y / (2*a_base_y))))))
                                    {
                                        mode = 2;
                                    }
                                    else
                                    {
                                        mode = 0;
                                    }
                                }
                                else if (((p1.x < p2.x) && (p2.x < p3.x)) || ((p1.x > p2.x) && (p2.x > p3.x)))
                                {
                                    mode = 1;
                                }
                                else if (((p1.y < p2.y) && (p2.y < p3.y)) || ((p1.y > p2.y) && (p2.y > p3.y)))
                                {
                                    mode = 2;
                                }
                                else
                                {
                                    mode = 0;
                                }
                            }
                            if (((p1.x != p2.x) && (p2.x != p3.x) && (p3.x != p1.x)) && ((p1.y == p2.y) || (p2.y == p3.y) || (p3.x == p1.y)))
                            {
                                mode = 1;
                            }
                            else if (((p1.y != p2.y) && (p2.y != p3.y) && (p3.x != p1.y)) && ((p1.x == p2.x) || (p2.x == p3.x) || (p3.x == p1.x)))
                            {
                                mode = 2;
                            }
                            else
                            {
                                mode = 0;
                            }

                            // if (true)
                            // {
                            //     float angle = atan(result[0]);
                            //     if ((std::fabs(angle - atan2(p3.y - p2.y , p3.x - p2.x) - 2*M_PI) >= (M_PI/2.)) && (std::fabs(angle - atan2(p3.y - p2.y , p3.x - p2.x)) >= (M_PI/2.)) && (std::fabs(angle - atan2(p3.y - p2.y , p3.x -p2.x) + 2*M_PI) >= (M_PI/2.)))
                            //     {
                            //         angle -= M_PI*(angle/std::fabs(angle));
                            //     }
                            //     float dist = (sqrtf(powf(p3.x - p2.x, 2.) + powf(p3.y - p2.y, 2.)) + sqrtf(powf(p2.x - p1.x, 2.) + powf(p2.y - p1.y, 2.))) / 0.5;
                            //     leg_point_temp.x = p3.x + dist * cos(angle);
                            //     leg_point_temp.y = p3.y + dist * sin(angle);
                            //     leg_point_temp.z = 0.4;
                            //     if (leg_point_temp.x < 1.5)
                            //     {
                            //         if (leg_point_temp.x < all_parameter.lidar_pose[0])
                            //         {
                            //             leg_point_temp.x = all_parameter.lidar_pose[0];
                            //             leg_point_temp.y = result[0] * leg_point_temp.x + result[1];
                            //         }
                            //         leg_points_steps.point_next.push_back(leg_point_temp);
                            //     }
                            // }
                            if (true)
                            {
                                float angle = atan(result[0]);
                                if ((std::fabs(angle - atan2(p3.y - p2.y , p3.x - p2.x) - 2*M_PI) >= (M_PI/2.)) && (std::fabs(angle - atan2(p3.y - p2.y , p3.x - p2.x)) >= (M_PI/2.)) && (std::fabs(angle - atan2(p3.y - p2.y , p3.x - p2.x) + 2*M_PI) >= (M_PI/2.)))
                                {
                                    angle -= M_PI*(angle/std::fabs(angle));
                                }
                                float dist = (sqrtf(powf(p3.x - p2.x, 2.) + powf(p3.y - p2.y, 2.)) + sqrtf(powf(p2.x - p1.x, 2.) + powf(p2.y - p1.y, 2.))) * (sqrtf(powf(p3.x, 2.) + powf(p3.y, 2.)) * all_parameter.x_weght_prediction);
                                leg_point_temp.x = p3.x + dist * cos(angle);
                                leg_point_temp.y = p3.y + dist * sin(angle);
                                leg_point_temp.z = 0.4;
                                if (leg_point_temp.x < (3.0 + 0.12))
                                {
                                    if (leg_point_temp.x < (all_parameter.lidar_pose[0] + 0.12))
                                    {
                                        leg_point_temp.x = all_parameter.lidar_pose[0] + 0.12;
                                        leg_point_temp.y = result[0] * leg_point_temp.x + result[1];
                                    }
                                    if (std::fabs(angle) <= 3*M_PI/4.)
                                    {
                                        dist = (sqrtf(powf(p3.x - p2.x, 2.) + powf(p3.y - p2.y, 2.)) + sqrtf(powf(p2.x - p1.x, 2.) + powf(p2.y - p1.y, 2.))) * (sqrtf(powf(p3.x, 2.) + powf(p3.y, 2.)) * all_parameter.y_weght_prediction);
                                        leg_point_temp.x = p3.x + dist * cos(angle);
                                        leg_point_temp.y = p3.y + dist * sin(angle);
                                    }
                                    leg_points_steps.point_next.push_back(leg_point_temp);
                                }
                            }
                            else if (mode == 1)
                            {
                                a = ((p1.y - p2.y) / ((p1.x - p2.x) * (p2.x - p3.x))) - ((p1.y - p3.y) / ((p1.x - p3.x) * (p2.x - p3.x)));
                                b = (p1.y - p2.y) / (p1.x - p2.x) - a * (p1.x + p2.x);
                                c = p1.y - a * powf(p1.x, 2.) - b * p1.x;
                                leg_point_temp.x = all_parameter.lidar_pose[0];
                                leg_point_temp.y = a * powf(leg_point_temp.x, 2.) + b * leg_point_temp.x + c;
                                leg_point_temp.z = 0.4;
                                leg_points_steps.point_next.push_back(leg_point_temp);
                            }
                            else if (mode == 2)
                            {
                                a = ((p1.x - p2.x) / ((p1.y - p2.y) * (p2.y - p3.y))) - ((p1.x - p3.x) / ((p1.y - p3.y) * (p2.y - p3.y)));
                                b = (p1.x - p2.x) / (p1.y - p2.y) - a * (p1.y + p2.y);
                                c = p1.x - a * powf(p1.y, 2.) - b * p1.y;
                                leg_point_temp.y = all_parameter.lidar_pose[1];
                                leg_point_temp.x = a * powf(leg_point_temp.y, 2.) + b * leg_point_temp.y + c;
                                leg_point_temp.z = 0.4;
                                leg_points_steps.point_next.push_back(leg_point_temp);
                            }
                            // leg_points_steps.point_next.push_back(leg_point_temp);
                        }
                    }
                }
                pub_walk_leg.publish(leg_points_steps);
                ros::spinOnce();
            }
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "leg_point_publisher");
    HUMAN_DETECT human_detect;
    ros::spin();
    return 0;
}