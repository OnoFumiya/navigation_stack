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

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <algorithm>

#include <Eigen/Dense>
using namespace Eigen;

// using namespace std;

typedef message_filters::sync_policies::ApproximateTime<dr_spaam_ros::LegPoseArray, sensor_msgs::LaserScan> DrspaamScanSyncPolicy;


class ALL_PARAMETER
{
    private:
        ros::NodeHandle nh_, pnh_;
    public:
        float lidar_pose[2] = {0.2, 0.0};
        // int sampling_step = 5;
        // float max_human_vel = 1.5;
        // float max_human_radius = 0.80;
        // float human_noise = 0.4;
        // float detect_range_time = 0.1;
        // float velocity_weight_prediction = 2.0;
        // float position_weight_prediction = 1.3333;
        // float similar_object = 0.2;
        // float no_moving_range = 0.6;
        // std::string robot_base = "base_footprint";
        // std::string map = "map";
        // std::string removal_scan_topic = "/dr_spaam_navigation/scan";
        // std::string stepping_point_topic = "/dr_spaam_navigation/object_pointers_step";
        // std::string dr_spaam_topic = "/dr_spaam_detections";
        // std::string merge_object_topic = "/object_point";
        // std::string removal_scan_marker = "/dr_spaam_navigation/lidar_points";
        // std::string step1_marker = "/dr_spaam_navigation/point_step1";
        // std::string step2_marker = "/dr_spaam_navigation/point_step2";
        // std::string step3_marker = "/dr_spaam_navigation/point_step3";
        // std::string next_step_marker = "/dr_spaam_navigation/point_nextstep";
        // std::string cost_topic = "/dr_spaam_navigation/point_cost";
        int sampling_step;
        float max_human_vel;
        float min_human_vel;
        float max_human_radius;
        float human_noise;
        float detect_range_time;
        float velocity_weight_prediction;
        float position_weight_prediction;
        float similar_object;
        float no_moving_range;
        std::string robot_base;
        std::string map;
        std::string removal_scan_topic;
        std::string stepping_point_topic;
        std::string dr_spaam_topic;
        std::string merge_object_topic;
        std::string removal_scan_marker;
        std::string step1_marker;
        std::string step2_marker;
        std::string step3_marker;
        std::string next_step_marker;
        std::string cost_topic;
        ALL_PARAMETER() : nh_(), pnh_("~")
        {
            sampling_step = nh_.param<float>( "sampling_step", 0.00 );
            max_human_vel = nh_.param<float>( "max_human_vel", 0.00 );
            min_human_vel = nh_.param<float>( "min_human_vel", 0.00 );
            max_human_radius = nh_.param<float>( "max_human_radius", 0.00 );
            human_noise = nh_.param<float>( "human_noise", 0.00 );
            velocity_weight_prediction = nh_.param<float>( "velocity_weight_prediction", 0.00 );
            position_weight_prediction = nh_.param<float>( "position_weight_prediction", 0.00 );
            similar_object = nh_.param<float>( "similar_object", 0.00 );
            no_moving_range = nh_.param<float>( "no_moving_range", 0.00 );
            detect_range_time = nh_.param<float>( "detect_range_time", 0.00 );
            robot_base = nh_.param<std::string>( "robot_base", "dummy" );
            map = nh_.param<std::string>( "map", "dummy" );
            removal_scan_topic = nh_.param<std::string>( "removal_scan_topic", "dummy" );
            stepping_point_topic = nh_.param<std::string>( "stepping_point_topic", "dummy" );
            dr_spaam_topic = nh_.param<std::string>( "dr_spaam_topic", "dummy" );
            merge_object_topic = nh_.param<std::string>( "merge_object_topic", "dummy" );
            removal_scan_marker = nh_.param<std::string>( "removal_scan_marker", "dummy" );
            step1_marker = nh_.param<std::string>( "step1_marker", "dummy" );
            step2_marker = nh_.param<std::string>( "step2_marker", "dummy" );
            step3_marker = nh_.param<std::string>( "step3_marker", "dummy" );
            next_step_marker = nh_.param<std::string>( "next_step_marker", "dummy" );
            cost_topic = nh_.param<std::string>( "cost_topic", "dummy" );
            // ros::NodeHandle nh;
            // XmlRpc::XmlRpcValue param;
            // nh.getParam("lidar_pose", param);
            // lidar_pose[0] = static_cast<double>(param[0]);
            // lidar_pose[1] = static_cast<double>(param[1]);
            // nh.getParam("sampling_step", param);
            // sampling_step = static_cast<int>(param);
            // nh.getParam("max_human_vel", param);
            // max_human_vel = static_cast<double>(param);
            // nh.getParam("max_human_radius", param);
            // max_human_radius = static_cast<double>(param);
            // nh.getParam("human_noise", param);
            // human_noise = static_cast<double>(param);
            // nh.getParam("velocity_weight_prediction", param);
            // velocity_weight_prediction = static_cast<double>(param);
            // nh.getParam("position_weight_prediction", param);
            // position_weight_prediction = static_cast<double>(param);
            // nh.getParam("similar_object", param);
            // similar_object = static_cast<double>(param);
            // nh.getParam("no_moving_range", param);
            // no_moving_range = static_cast<double>(param);
            // nh.getParam("detect_range_time", param);
            // detect_range_time = static_cast<double>(param);
            // nh.getParam("robot_base", param);
            // robot_base = static_cast<std::string>(param);
            // nh.getParam("map", param);
            // map = static_cast<std::string>(param);
            // nh.getParam("removal_scan_topic", param);
            // removal_scan_topic = static_cast<std::string>(param);
            // nh.getParam("stepping_point_topic", param);
            // stepping_point_topic = static_cast<std::string>(param);
            // nh.getParam("dr_spaam_topic", param);
            // dr_spaam_topic = static_cast<std::string>(param);
            // nh.getParam("merge_object_topic", param);
            // merge_object_topic = static_cast<std::string>(param);
            // nh.getParam("removal_scan_marker", param);
            // removal_scan_marker = static_cast<std::string>(param);
            // nh.getParam("step1_marker", param);
            // step1_marker = static_cast<std::string>(param);
            // nh.getParam("step2_marker", param);
            // step2_marker = static_cast<std::string>(param);
            // nh.getParam("step3_marker", param);
            // step3_marker = static_cast<std::string>(param);
            // nh.getParam("next_step_marker", param);
            // next_step_marker = static_cast<std::string>(param);
            // nh.getParam("cost_topic", param);
            // cost_topic = static_cast<std::string>(param);
        }
};


class OBJECT_DETECT
{
    private:
        ros::NodeHandle nh;
        tf::TransformListener tf_listener;
        ros::Publisher pub_step_point;
        ros::Publisher pub_pointremoval_scan;
        ros::Subscriber object_sub;
        ros::Subscriber robotvel_sub;
        ALL_PARAMETER all_parameter;
        bool start_flag;
        std::vector<geometry_msgs::Point> object_points;
        std::vector<geometry_msgs::Point> points_base;
        geometry_msgs::Twist robot_vel;

        std::unique_ptr<message_filters::Subscriber<dr_spaam_ros::LegPoseArray>> sub_legs_;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>>     sub_scan_;

        std::shared_ptr<message_filters::Synchronizer<DrspaamScanSyncPolicy>>    sync_;
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
        // void callback_object(const dr_spaam_ros::LegPoseArray &msg)
        void callback_object(const dr_spaam_ros::LegPoseArrayConstPtr &msg, 
                             const sensor_msgs::LaserScanConstPtr     &scan_msg)
        {
            object_points.clear();
            points_base.clear();
            geometry_msgs::Point point;
            for (int i=0; i<msg->poses.size(); i++)
            {
                point.x = msg->poses[i].position.x + all_parameter.lidar_pose[0];
                point.y = msg->poses[i].position.y + all_parameter.lidar_pose[1];
                point.z = 0.2;
                if ((0.25 <= sqrtf(powf(point.x, 2.) + powf(point.y, 2.))) && (sqrtf(powf(point.x, 2.) + powf(point.y, 2.)) <= scan_msg->range_max))
                {
                    points_base.push_back(point);
                    object_points.push_back(Pointtransform(all_parameter.robot_base, all_parameter.map, point));
                }
            }

            sensor_msgs::LaserScan removal_lidar_data;
            removal_lidar_data.header.seq = scan_msg->header.seq;
            removal_lidar_data.header.stamp = scan_msg->header.stamp;
            removal_lidar_data.header.frame_id = scan_msg->header.frame_id;
            removal_lidar_data.angle_min = scan_msg->angle_min;
            removal_lidar_data.angle_max = scan_msg->angle_max;
            removal_lidar_data.angle_increment = scan_msg->angle_increment;
            removal_lidar_data.time_increment = scan_msg->time_increment;
            removal_lidar_data.scan_time = scan_msg->scan_time;
            removal_lidar_data.range_min = scan_msg->range_min;
            removal_lidar_data.range_max = scan_msg->range_max;
            removal_lidar_data.ranges.resize(scan_msg->ranges.size());
            copy(scan_msg->ranges.begin(), scan_msg->ranges.end(), removal_lidar_data.ranges.begin());
            for (int i=0; i<scan_msg->ranges.size(); i++)
            {
                if ((scan_msg->range_min <= scan_msg->ranges[i]) && (scan_msg->ranges[i] <= scan_msg->range_max))
                {
                    for (int j=0; j<points_base.size(); j++)
                    {
                        if (sqrtf(powf((scan_msg->ranges[i]*(cos(scan_msg->angle_min + scan_msg->angle_increment*i)) + all_parameter.lidar_pose[0]) - points_base[j].x, 2.) + powf((scan_msg->ranges[i]*(sin(scan_msg->angle_min + scan_msg->angle_increment*i)) + all_parameter.lidar_pose[1]) - points_base[j].y, 2.)) <= all_parameter.max_human_radius)
                        {
                            removal_lidar_data.ranges[i] = NAN;
                            break;
                        }
                    }
                }
            }
            pub_pointremoval_scan.publish(removal_lidar_data);
            start_flag = true;
        }
        void callback_robotvel(const geometry_msgs::Twist &msg)
        {
            robot_vel.linear.x = msg.linear.x;
            robot_vel.linear.y = msg.linear.y;
            robot_vel.linear.z = msg.linear.z;
            robot_vel.angular.x = msg.angular.x;
            robot_vel.angular.y = msg.angular.y;
            robot_vel.angular.z = msg.angular.z;
        }
    public:
        OBJECT_DETECT()
        {
            robotvel_sub = nh.subscribe("mobile_base/commands/velocity", 10, &OBJECT_DETECT::callback_robotvel, this);
            pub_step_point = nh.advertise<navigation_stack::WalkLegPoint>(all_parameter.stepping_point_topic, 10);
            pub_pointremoval_scan = nh.advertise<sensor_msgs::LaserScan>(all_parameter.removal_scan_topic, 10);
            // object_sub = nh.subscribe(all_parameter.merge_object_topic, 10, &OBJECT_DETECT::callback_object, this);

            sub_legs_.reset(new message_filters::Subscriber<dr_spaam_ros::LegPoseArray>(nh, all_parameter.merge_object_topic, 5));
            sub_scan_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, "/scan", 5));
            
            sync_.reset(new message_filters::Synchronizer<DrspaamScanSyncPolicy>(DrspaamScanSyncPolicy(200), *sub_legs_, *sub_scan_));
            sync_->registerCallback(boost::bind(&OBJECT_DETECT::callback_object, this, _1, _2));

            robot_vel.linear.x = 0.;
            robot_vel.linear.y = 0.;
            robot_vel.linear.z = 0.;
            robot_vel.angular.x = 0.;
            robot_vel.angular.y = 0.;
            robot_vel.angular.z = 0.;
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
            navigation_stack::WalkLegPoint points_steps;
            points_steps.point1.resize(object_points.size());
            points_steps.point2.resize(object_points.size());
            points_steps.point3.resize(object_points.size());
            copy(object_points.begin(), object_points.end(), points_steps.point1.begin());
            copy(object_points.begin(), object_points.end(), points_steps.point2.begin());
            copy(object_points.begin(), object_points.end(), points_steps.point3.begin());
            points_steps.point_next.clear();
            while (ros::ok())
            {
                points_steps.point1.clear();
                points_steps.point1.resize(points_steps.point2.size());
                copy(points_steps.point2.begin(), points_steps.point2.end(), points_steps.point1.begin());
                points_steps.point2.clear();
                points_steps.point2.resize(points_steps.point3.size());
                copy(points_steps.point3.begin(), points_steps.point3.end(), points_steps.point2.begin());

                std::vector<std::vector<geometry_msgs::Point>> object_points_samplings;   // 第一要素が物体の数、第二要素が物体のそれぞれのステップ
                std::vector<geometry_msgs::Point> object_points_temp;
                object_points_samplings.clear();
                object_points_samplings.resize(object_points.size());
                object_points_temp.resize(object_points.size());
                copy(object_points.begin(), object_points.end(), object_points_temp.begin());
                for (int i=0; i<object_points_temp.size(); i++)
                {
                    object_points_samplings[i].push_back(object_points_temp[i]);
                }
                for (int i=1; i<all_parameter.sampling_step; i++)
                {
                    ros::Duration(all_parameter.detect_range_time).sleep();
                    object_points_temp.resize(object_points.size());
                    copy(object_points.begin(), object_points.end(), object_points_temp.begin());
                    for (int j=0; j<object_points_temp.size(); j++)
                    {
                        bool input_flag = false;
                        for (int k=0; k<object_points_samplings.size(); k++)
                        {
                            if (sqrtf(powf(object_points_temp[j].x - object_points_samplings[k][object_points_samplings[k].size()-1].x, 2.) + powf(object_points_temp[j].y - object_points_samplings[k][object_points_samplings[k].size()-1].y, 2.)) <= all_parameter.human_noise)
                            {
                                object_points_samplings[k].push_back(object_points_temp[j]);
                                input_flag = true;
                            }
                        }
                        if (input_flag != true)
                        {
                            std::vector<geometry_msgs::Point> new_input_object;
                            new_input_object.clear();
                            new_input_object.push_back(object_points_temp[j]);
                            object_points_samplings.push_back(new_input_object);
                        }
                    }
                }

                geometry_msgs::Point ob_point_temp;
                geometry_msgs::Point pt_nan;
                pt_nan.x = NAN;
                pt_nan.y = NAN;
                pt_nan.z = 0.0;
                points_steps.point3.clear();
                points_steps.point3.resize(points_steps.point2.size(), pt_nan);
                for (int i=0; i<object_points_samplings.size(); i++)
                {
                    float x = 0.0;
                    float y = 0.0;
                    for (int j=0; j<object_points_samplings[i].size(); j++)
                    {
                        x += object_points_samplings[i][j].x;
                        y += object_points_samplings[i][j].y;
                    }
                    ob_point_temp.x = x/object_points_samplings[i].size();
                    ob_point_temp.y = y/object_points_samplings[i].size();
                    ob_point_temp.z = 0.2;
                    bool input_flag = false;
                    for (int j=0; j<points_steps.point2.size(); j++)
                    {
                        if (sqrtf(powf(ob_point_temp.x - points_steps.point2[j].x, 2.) + powf(ob_point_temp.y - points_steps.point2[j].y, 2.)) <= all_parameter.max_human_vel*all_parameter.detect_range_time*all_parameter.sampling_step)
                        {
                            points_steps.point3[j].x = ob_point_temp.x;
                            points_steps.point3[j].y = ob_point_temp.y;
                            points_steps.point3[j].z = ob_point_temp.z;
                            input_flag = true;
                            break;
                        }
                    }
                    if (input_flag != true)
                    {
                        points_steps.point3.push_back(ob_point_temp);
                    }
                }

                for (int i=0; i<points_steps.point3.size(); i++)
                {
                    if ((std::isnan(points_steps.point3[i].x)) || (std::isnan(points_steps.point3[i].y)))
                    {
                        points_steps.point3.erase(points_steps.point3.begin() + i);
                        if ((0 <= i) && (i < points_steps.point1.size()))
                        {
                            points_steps.point1.erase(points_steps.point1.begin() + i);
                        }
                        if ((0 <= i) && (i < points_steps.point2.size()))
                        {
                            points_steps.point2.erase(points_steps.point2.begin() + i);
                        }
                    }
                }

                geometry_msgs::Point p1, p2, p3;
                int mode = 0;
                float a, b, c;
                points_steps.point_next.clear();
                for (int i=0; i<points_steps.point3.size(); i++)
                {
                    if ((i < points_steps.point1.size()) && (i < points_steps.point2.size()))
                    {
                        if ((std::isnan(points_steps.point1[i].x) != true) && (std::isnan(points_steps.point1[i].y) != true) && (std::isnan(points_steps.point2[i].x) != true) && (std::isnan(points_steps.point2[i].y) != true) && (std::isnan(points_steps.point3[i].x) != true) && (std::isnan(points_steps.point3[i].y) != true))
                        {
                            p1 = Pointtransform(all_parameter.map, all_parameter.robot_base, points_steps.point1[i]);
                            p2 = Pointtransform(all_parameter.map, all_parameter.robot_base, points_steps.point2[i]);
                            p3 = Pointtransform(all_parameter.map, all_parameter.robot_base, points_steps.point3[i]);
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
                            if ((all_parameter.min_human_vel <= sqrtf(powf((p3.x - p1.x)/2, 2) + powf((p3.y - p1.y)/2, 2))/all_parameter.detect_range_time) && (sqrtf(powf((p3.x - p1.x)/2, 2) + powf((p3.y - p1.y)/2, 2))/all_parameter.detect_range_time <= all_parameter.max_human_vel))
                            {
                                float angle = atan(result[0]);
                                if ((std::fabs(angle - atan2(p3.y - p2.y , p3.x - p2.x) - 2*M_PI) >= (M_PI/2.)) && (std::fabs(angle - atan2(p3.y - p2.y , p3.x - p2.x)) >= (M_PI/2.)) && (std::fabs(angle - atan2(p3.y - p2.y , p3.x - p2.x) + 2*M_PI) >= (M_PI/2.)))
                                {
                                    angle -= M_PI*(angle/std::fabs(angle));
                                }
                                if ((std::fabs(angle) <= M_PI/4) || (3*M_PI/4 <= std::fabs(angle)))
                                {
                                    ob_point_temp.x = p3.x - 1.3 * std::fabs(p3.x) * (((p3.x - p2.x) + (p2.x - p1.x)) / (2*all_parameter.detect_range_time)) / ((((p3.x - p2.x) + (p2.x - p1.x)) / (2*all_parameter.detect_range_time)) - sqrtf(pow(robot_vel.linear.x, 2.) + pow(robot_vel.linear.y, 2.)));
                                    ob_point_temp.y = result[0] * ob_point_temp.x + result[1];
                                    if (ob_point_temp.x <= all_parameter.lidar_pose[0] + 0.1)
                                    {
                                        ob_point_temp.x = all_parameter.lidar_pose[0] + 0.1;
                                        ob_point_temp.y = result[0] * ob_point_temp.x + result[1];
                                    }
                                }
                                else
                                {
                                    ob_point_temp.y = p3.y - std::fabs(p3.y) * (((p3.y - p2.y) + (p2.y - p1.y)) / (2*all_parameter.detect_range_time)) / ((((p3.y - p2.y) + (p2.y - p1.y)) / (2*all_parameter.detect_range_time)) - sqrtf(pow(robot_vel.linear.x, 2.) + pow(robot_vel.linear.y, 2.)));
                                    ob_point_temp.x = (ob_point_temp.y - result[1]) / result[0];
                                }
                                points_steps.point_next.push_back(ob_point_temp);
                            }
                            else
                            {
                                ob_point_temp.x = p3.x;
                                ob_point_temp.y = p3.y;
                                points_steps.point_next.push_back(ob_point_temp);
                            }
                        }
                    }
                }
                pub_step_point.publish(points_steps);
                ros::spinOnce();
            }
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "step_point_publisher");
    OBJECT_DETECT object_detect;
    ros::spin();
    return 0;
}