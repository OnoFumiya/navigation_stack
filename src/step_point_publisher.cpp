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
#include <std_srvs/Empty.h>
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
        int clear_per_times;
        std::vector<double> trajectry_weight;
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
            clear_per_times = nh_.param<int>( "clear_per_times", 1 );
            trajectry_weight = nh_.param<std::vector<double>>( "trajectry_weight", {0.1, 0.1, 0.1, 0.1, 0.1} );
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
        ros::ServiceClient clt_clear_costmaps;
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

        // 近似直線
        void line_path(const std::vector<geometry_msgs::Point> pts, 
                       double &a, double &b, bool &flag, 
                       std::vector<geometry_msgs::Point> &s_pts) {
            // 例外
            flag = true;
            if ((3*(std::pow(pts[0].x, 2) + std::pow(pts[1].x, 2) + std::pow(pts[2].x, 2))) == std::pow(pts[0].x + pts[1].x + pts[2].x, 2)) flag = false;

            // 算出
            if (flag) {
                double sum_x, sum_y, sum_xx, sum_xy;
                sum_x = pts[0].x + pts[1].x + pts[2].x;
                sum_y = pts[0].y + pts[1].y + pts[2].y;
                sum_xx = std::pow(pts[0].x, 2) + std::pow(pts[1].x, 2) + std::pow(pts[2].x, 2);
                sum_xy = pts[0].x*pts[0].y + pts[1].x*pts[1].y + pts[2].x*pts[2].y;
                a = (3.0 * sum_xy - sum_x * sum_y) / (3.0 * sum_xx - sum_x * sum_x);
                b = (sum_y - a * sum_x) / 3.0;
            }

            // 棄却許容判定
            if (flag) {
                // double samp_x1, samp_x2, samp_x3, samp_y1, samp_y2, samp_y3;
                s_pts[0].x = (-a*b + a*pts[0].y + pts[0].x) / (std::pow(a, 2.) + 1.);
                s_pts[0].y = (-a*b + a*pts[0].y + pts[0].x) / (std::pow(a, 2.) + 1.) + b;
                s_pts[1].x = (-a*b + a*pts[1].y + pts[1].x) / (std::pow(a, 2.) + 1.);
                s_pts[1].y = (-a*b + a*pts[1].y + pts[1].x) / (std::pow(a, 2.) + 1.) + b;
                s_pts[2].x = (-a*b + a*pts[2].y + pts[2].x) / (std::pow(a, 2.) + 1.);
                s_pts[2].y = (-a*b + a*pts[2].y + pts[2].x) / (std::pow(a, 2.) + 1.) + b;
                if (((((s_pts[0].x <= s_pts[1].x) && (s_pts[1].x <= s_pts[2].x)) || ((s_pts[2].x <= s_pts[1].x) && (s_pts[1].x <= s_pts[0].x))) && (s_pts[2].x != s_pts[0].x)) ||
                    ((((s_pts[0].y <= s_pts[1].y) && (s_pts[1].y <= s_pts[2].y)) || ((s_pts[2].y <= s_pts[1].y) && (s_pts[1].y <= s_pts[0].y))) && (s_pts[2].y != s_pts[0].y))) {
                        flag = true;
                } else  flag = false;
            }
        }

        // 放物線
        void parabola(const std::vector<geometry_msgs::Point> pts, 
                      double &ax, double &bx, double &cx, bool &flag_x, double &ay, double &by, double &cy, bool &flag_y) {
            // 例外
            flag_x = true;
            flag_y = true;
            if ((pts[0].x == pts[1].x) || (pts[1].x == pts[2].x) || (pts[2].x == pts[0].x)) flag_x = false;
            if ((pts[0].y == pts[1].y) || (pts[1].y == pts[2].y) || (pts[2].y == pts[0].y)) flag_y = false;

            // 算出
            if (flag_x) {
                ax = (pts[0].x*(pts[2].y - pts[1].y) + pts[1].x*(pts[0].y - pts[2].y) + pts[2].x*(pts[1].y - pts[0].y)) / ((pts[0].x - pts[1].x) * (pts[0].x - pts[2].x) * (pts[1].x - pts[2].x));
                bx = ((std::pow(pts[1].x, 2) - std::pow(pts[2].x, 2))*pts[0].y + (std::pow(pts[2].x, 2) - std::pow(pts[0].x, 2))*pts[1].y + (std::pow(pts[0].x, 2) - std::pow(pts[1].x, 2))*pts[2].y) / ((pts[0].x - pts[1].x) * (pts[0].x - pts[2].x) * (pts[2].x - pts[1].x));
                cx = pts[0].y - ax*std::pow(pts[0].x, 2) -bx*pts[0].x;
            }
            if (flag_y) {
                ay= (pts[0].y*(pts[2].x - pts[1].x) + pts[1].y*(pts[0].x - pts[2].x) + pts[2].y*(pts[1].x - pts[0].x)) / ((pts[0].y - pts[1].y) * (pts[0].y - pts[2].y) * (pts[1].y - pts[2].y));
                by= ((std::pow(pts[1].y, 2) - std::pow(pts[2].y, 2))*pts[0].x + (std::pow(pts[2].y, 2) - std::pow(pts[0].y, 2))*pts[1].x + (std::pow(pts[0].y, 2) - std::pow(pts[1].y, 2))*pts[2].x) / ((pts[0].y - pts[1].y) * (pts[0].y - pts[2].y) * (pts[2].y - pts[1].y));
                cy= pts[0].x - ay*std::pow(pts[0].y, 2) -by*pts[0].y;
            }

            // 棄却許容判定
            if (flag_x) {
                if ((((pts[0].x <= pts[1].x) && (pts[1].x <= pts[2].x)) || ((pts[2].x <= pts[1].x) && (pts[1].x <= pts[0].x))) && (pts[2].x != pts[0].x)) {
                        flag_x = true;
                } else  flag_x = false;
                double pre_x, pre_y;
                pre_x = ( ((std::pow(2*ax*pts[2].x+bx,2) + 1)*pts[2].x) + ((pts[2].x-pts[1].x)/std::fabs(pts[2].x-pts[1].x)) * std::sqrt(std::pow((std::pow(2*ax*pts[2].x+bx,2) + 1)*pts[2].x,2) - (std::pow(2*ax*pts[2].x+bx,2) + 1)*(std::pow(pts[2].x,2)*(std::pow(2*ax*pts[2].x+bx,2) + 1)-1)) ) / ( std::pow(2*ax*pts[2].x+bx,2) + 1 );
                pre_y = (2*ax*pts[2].x + bx)*pre_x + pts[2].y - (2*ax*pts[2].x + bx) * pts[2].x;
                if ((((pts[0].x < pts[2].x) && (pts[2].x < -(bx)/(2*ax))) || ((-(bx)/(2*ax) < pts[2].x) && (pts[2].x < pts[0].x))) || 
                    ( acos(   ( (pts[2].x-pts[1].x) * (pre_x-pts[2].x) + (pts[2].y-pts[1].y) * (pre_y-pts[2].y) ) / std::sqrt(std::pow(pts[2].x-pts[1].x, 2) + std::pow(pts[2].y-pts[1].y, 2))   ) > M_PI/2. )) {
                    flag_x = false;
                }
            }
            if (flag_y) {
                if ((((pts[0].y <= pts[1].y) && (pts[1].y <= pts[2].y)) || ((pts[2].y <= pts[1].y) && (pts[1].y <= pts[0].y))) && (pts[2].y != pts[0].y)) {
                        flag_y = true;
                } else  flag_y = false;
                double pre_x, pre_y;
                pre_y = ( ((std::pow(2*ay*pts[2].y+by,2) + 1)*pts[2].y) + ((pts[2].y-pts[1].y)/std::fabs(pts[2].y-pts[1].y)) * std::sqrt(std::pow((std::pow(2*ay*pts[2].y+by,2) + 1)*pts[2].y,2) - (std::pow(2*ay*pts[2].y+by,2) + 1)*(std::pow(pts[2].y,2)*(std::pow(2*ay*pts[2].y+by,2) + 1)-1)) ) / ( std::pow(2*ay*pts[2].y+by,2) + 1 );
                pre_x = (2*ay*pts[2].y + by)*pre_y + pts[2].x - (2*ay*pts[2].y + by) * pts[2].y;
                if ((((pts[0].y < pts[2].y) && (pts[2].y < -(by)/(2*ay))) || ((-(by)/(2*ay) < pts[2].y) && (pts[2].y < pts[0].y))) || 
                    ( acos(   ( (pts[2].y-pts[1].y) * (pre_y-pts[2].y) + (pts[2].x-pts[1].x) * (pre_x-pts[2].x) ) / std::sqrt(std::pow(pts[2].y-pts[1].y, 2) + std::pow(pts[2].x-pts[1].x, 2))   ) > M_PI/2. )) {
                    flag_y = false;
                }
            }
        }

        // シグモイド曲線
        void sigmoid(const std::vector<geometry_msgs::Point> pts, 
                     double &Lx, double &ax, double &dxx, double &dyx, bool &flag_x, double &Ly, double &ay, double &dxy, double &dyy, bool &flag_y) {
            // 例外
            flag_x = true;
            flag_y = true;
            if ((pts[2].y == pts[1].y) || (pts[2].x == pts[1].x)) {
                flag_x = false;
                flag_y = false;
            }

            // 算出
            if (flag_x && flag_y) {
                dxx = pts[1].x;
                Lx = 2 * std::fabs(pts[2].y-pts[1].y);
                dyx = pts[1].y - (Lx / 2.0);
                ax = 4 * ((pts[2].y - pts[1].y) / (pts[2].x - pts[1].x)) / Lx;

                dyy = pts[1].y;
                Ly = 2 * std::fabs(pts[2].x-pts[1].x);
                dxy = pts[1].x - (Ly / 2.0);
                ay = 4 * ((pts[2].x - pts[1].x) / (pts[2].y - pts[1].y)) / Ly;
            }

            // 棄却許容判定
            if (flag_x && flag_y) {
                if ((((pts[0].x <= pts[1].x) && (pts[1].x <= pts[2].x)) || ((pts[2].x <= pts[1].x) && (pts[1].x <= pts[0].x))) && (pts[2].x != pts[0].x)) {
                        flag_x = true;
                } else  flag_x = false;
                if ((((pts[0].y <= pts[1].y) && (pts[1].y <= pts[2].y)) || ((pts[2].y <= pts[1].y) && (pts[1].y <= pts[0].y))) && (pts[2].y != pts[0].y)) {
                        flag_y = true;
                } else  flag_y = false;
                if (flag_x) {
                    if ( (( atan((pts[2].y-pts[0].y) / (pts[2].x-pts[0].x))/2. + atan((pts[2].y-pts[1].y) / (pts[2].x-pts[1].x))/2. ) < (-5*M_PI/6.)) && ((5*M_PI/6.) < ( atan((pts[2].y-pts[0].y) / (pts[2].x-pts[0].x))/2. + atan((pts[2].y-pts[1].y) / (pts[2].x-pts[1].x))/2. )) ) flag_x = false;
                }
                if (flag_y) {
                    if ( (-M_PI/6.) < (( atan((pts[2].y-pts[0].y) / (pts[2].x-pts[0].x))/2. + atan((pts[2].y-pts[1].y) / (pts[2].x-pts[1].x))/2. )) && (( atan((pts[2].y-pts[0].y) / (pts[2].x-pts[0].x))/2. + atan((pts[2].y-pts[1].y) / (pts[2].x-pts[1].x))/2. ) < (M_PI/6.)) ) flag_y = false;
                }
            }
        }

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
            robotvel_sub = nh.subscribe("/mobile_base/commands/velocity", 1, &OBJECT_DETECT::callback_robotvel, this);
            pub_step_point = nh.advertise<navigation_stack::WalkLegPoint>(all_parameter.stepping_point_topic, 10);
            pub_pointremoval_scan = nh.advertise<sensor_msgs::LaserScan>(all_parameter.removal_scan_topic, 10);
            clt_clear_costmaps = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

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
            std_srvs::Empty emp_srv;
            int counter=1;
            while (ros::ok())
            {
                ros::spinOnce();
                // 新たなステップへ更新
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
                for (int i=1; i<=all_parameter.sampling_step; i++)
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
                if (0 < all_parameter.clear_per_times) {
                    if (all_parameter.clear_per_times <= counter) {
                        clt_clear_costmaps.call(emp_srv);
                        counter = 1;
                    }
                    else counter++;
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
                // 新たなステップへ更新 //

                // 軌道予測
                geometry_msgs::Point p1, p2, p3;
                points_steps.point_next.clear();
                for (int i=0; i<points_steps.point3.size(); i++) {
                    // 例外処理
                    if ((points_steps.point1.size() <= i) || (points_steps.point2.size() <= i)) continue;
                    if ((std::isnan(points_steps.point1[i].x)) || (std::isnan(points_steps.point1[i].y)) || (std::isnan(points_steps.point2[i].x)) || (std::isnan(points_steps.point2[i].y)) || (std::isnan(points_steps.point3[i].x)) || (std::isnan(points_steps.point3[i].y))) continue;
                    p1 = Pointtransform(all_parameter.map, all_parameter.robot_base, points_steps.point1[i]);
                    p2 = Pointtransform(all_parameter.map, all_parameter.robot_base, points_steps.point2[i]);
                    p3 = Pointtransform(all_parameter.map, all_parameter.robot_base, points_steps.point3[i]);
                    if ((sqrtf(powf(p3.x - p2.x, 2) + powf(p3.y - p2.y, 2))/(2*all_parameter.detect_range_time) < all_parameter.min_human_vel) || (all_parameter.max_human_vel < sqrtf(powf(p3.x - p2.x, 2) + powf(p3.y - p2.y, 2))/(2*all_parameter.detect_range_time))) {
                        points_steps.point_next.push_back(p3);
                        continue;
                    }
                    // 3つの軌道予測（type1:近似直線，type2:x軸が変数の放物線，type3:y軸が変数の放物線，type4:x軸が変数のシグモイド曲線，type5:y軸が変数のシグモイド曲線）
                    std::vector<geometry_msgs::Point> pts_s_1d({p1, p2, p3});
                    double a_1d, b_1b;
                    double a_2d, b_2d, c_2d;
                    double ta_2d, tb_2d, tc_2d;
                    double L_sig, a_sig, dx_sig, dy_sig;
                    double tL_sig, ta_sig, tdx_sig, tdy_sig;
                    bool check_type1, check_type2, check_type3, check_type4, check_type5;
                    line_path({p1, p2, p3}, a_1d, b_1b, check_type1, pts_s_1d);
                    parabola({p1, p2, p3}, a_2d, b_2d, c_2d, check_type2, ta_2d, tb_2d, tc_2d, check_type3);
                    sigmoid({p1, p2, p3}, L_sig, a_sig, dx_sig, dy_sig, check_type4, tL_sig, ta_sig, tdx_sig, tdy_sig, check_type5);
                    double sum_weight = 0.;
                    if (check_type1) sum_weight += all_parameter.trajectry_weight[0];
                    if (check_type2) sum_weight += all_parameter.trajectry_weight[1];
                    if (check_type3) sum_weight += all_parameter.trajectry_weight[2];
                    if (check_type4) sum_weight += all_parameter.trajectry_weight[3];
                    if (check_type5) sum_weight += all_parameter.trajectry_weight[4];
                    if (sum_weight == 0.) continue;
                    double dist_robot_ob = std::sqrt(std::pow(p3.x-0., 2.) + std::pow(p3.y-0., 2.));
                    geometry_msgs::Point next_pt;
                    next_pt.x = p3.x;
                    next_pt.y = p3.y;
                    next_pt.z = 0.3;
                    int step;
                    for (step=1; ; step++) {
                        geometry_msgs::Point robot_pt;
                        robot_pt.x = robot_vel.linear.x * all_parameter.detect_range_time * (step + 1) + all_parameter.lidar_pose[0];
                        robot_pt.y = robot_vel.linear.y * all_parameter.detect_range_time * (step + 1) + all_parameter.lidar_pose[1];
                        robot_pt.z = 0.;

                        geometry_msgs::Point ob_pt_type1, ob_pt_type2, ob_pt_type3, ob_pt_type4, ob_pt_type5, sampling_pt;
                        if (check_type1) {
                            ob_pt_type1.x = p3.x + (pts_s_1d[2].x - pts_s_1d[0].x) / std::fabs(pts_s_1d[2].x - pts_s_1d[0].x) * all_parameter.detect_range_time * step;
                            ob_pt_type1.y = a_1d * ob_pt_type1.x + b_1b;
                            ob_pt_type1.z = 0.;
                        }
                        if (check_type2) {
                            ob_pt_type2.x = p3.x + (p3.x - p1.x) / std::fabs(p3.x - p1.x) * all_parameter.detect_range_time * step;
                            ob_pt_type2.y = a_2d * std::pow(ob_pt_type2.x, 2.) + b_2d * ob_pt_type2.x + c_2d;
                            ob_pt_type2.z = 0.;
                        }
                        if (check_type3) {
                            ob_pt_type3.y = p3.y + (p3.y - p1.y) / std::fabs(p3.y - p1.y) * all_parameter.detect_range_time * step;
                            ob_pt_type3.x = ta_2d * std::pow(ob_pt_type3.y, 2.) + tb_2d * ob_pt_type3.y + tc_2d;
                            ob_pt_type3.z = 0.;
                        }
                        if (check_type4) {
                            ob_pt_type4.x = p3.x + (p3.x - p2.x) / std::fabs(p3.x - p2.x) * all_parameter.detect_range_time * step;
                            ob_pt_type4.y = L_sig / (1 + exp(-a_sig * (ob_pt_type4.x - dx_sig))) + dy_sig;
                            ob_pt_type4.z = 0.;
                        }
                        if (check_type5) {
                            ob_pt_type5.y = p3.y + (p3.y - p2.y) / std::fabs(p3.y - p2.y) * all_parameter.detect_range_time * step;
                            ob_pt_type5.x = tL_sig / (1 + exp(-ta_sig * (ob_pt_type5.y - tdy_sig))) + tdx_sig;
                            ob_pt_type5.z = 0.;
                        }

                        sampling_pt.x = 0.;
                        sampling_pt.y = 0.;
                        sampling_pt.z = 0.;
                        if (check_type1) {
                            sampling_pt.x += ob_pt_type1.x * (all_parameter.trajectry_weight[0] / sum_weight);
                            sampling_pt.y += ob_pt_type1.y * (all_parameter.trajectry_weight[0] / sum_weight);
                        }
                        if (check_type2) {
                            sampling_pt.x += ob_pt_type2.x * (all_parameter.trajectry_weight[1] / sum_weight);
                            sampling_pt.y += ob_pt_type2.y * (all_parameter.trajectry_weight[1] / sum_weight);
                        }
                        if (check_type3) {
                            sampling_pt.x += ob_pt_type3.x * (all_parameter.trajectry_weight[2] / sum_weight);
                            sampling_pt.y += ob_pt_type3.y * (all_parameter.trajectry_weight[2] / sum_weight);
                        }
                        if (check_type4) {
                            sampling_pt.x += ob_pt_type4.x * (all_parameter.trajectry_weight[3] / sum_weight);
                            sampling_pt.y += ob_pt_type4.y * (all_parameter.trajectry_weight[3] / sum_weight);
                        }
                        if (check_type5) {
                            sampling_pt.x += ob_pt_type5.x * (all_parameter.trajectry_weight[4] / sum_weight);
                            sampling_pt.y += ob_pt_type5.y * (all_parameter.trajectry_weight[4] / sum_weight);
                        }
                        if (std::sqrt(std::pow(sampling_pt.x - robot_pt.x, 2.) + std::pow(sampling_pt.y - robot_pt.y, 2.)) < dist_robot_ob) {
                            dist_robot_ob = std::sqrt(std::pow(sampling_pt.x - robot_pt.x, 2.) + std::pow(sampling_pt.y - robot_pt.y, 2.));
                            next_pt.x = sampling_pt.x;
                            next_pt.y = sampling_pt.y;
                        }
                        else break;
                    }
                    if (step == 1) {
                        geometry_msgs::Point robot_pt;
                        robot_pt.x = -robot_vel.linear.x * all_parameter.detect_range_time + all_parameter.lidar_pose[0];
                        robot_pt.y = -robot_vel.linear.y * all_parameter.detect_range_time + all_parameter.lidar_pose[1];
                        robot_pt.z = 0.;

                        geometry_msgs::Point ob_pt_type1, ob_pt_type2, ob_pt_type3, ob_pt_type4, ob_pt_type5, sampling_pt;
                        if (check_type1) {
                            ob_pt_type1.x = p3.x - (pts_s_1d[2].x - pts_s_1d[0].x) / std::fabs(pts_s_1d[2].x - pts_s_1d[0].x) * all_parameter.detect_range_time;
                            ob_pt_type1.y = a_1d * ob_pt_type1.x + b_1b;
                            ob_pt_type1.z = 0.;
                        }
                        if (check_type2) {
                            ob_pt_type2.x = p3.x - (p3.x - p1.x) / std::fabs(p3.x - p1.x) * all_parameter.detect_range_time;
                            ob_pt_type2.y = a_2d * std::pow(ob_pt_type2.x, 2.) + b_2d * ob_pt_type2.x + c_2d;
                            ob_pt_type2.z = 0.;
                        }
                        if (check_type3) {
                            ob_pt_type3.y = p3.y - (p3.y - p1.y) / std::fabs(p3.y - p1.y) * all_parameter.detect_range_time;
                            ob_pt_type3.x = ta_2d * std::pow(ob_pt_type3.y, 2.) + tb_2d * ob_pt_type3.y + tc_2d;
                            ob_pt_type3.z = 0.;
                        }
                        if (check_type4) {
                            ob_pt_type4.x = p3.x - (p3.x - p2.x) / std::fabs(p3.x - p2.x) * all_parameter.detect_range_time;
                            ob_pt_type4.y = L_sig / (1 + exp(-a_sig * (ob_pt_type4.x - dx_sig))) + dy_sig;
                            ob_pt_type4.z = 0.;
                        }
                        if (check_type5) {
                            ob_pt_type5.y = p3.y - (p3.y - p2.y) / std::fabs(p3.y - p2.y) * all_parameter.detect_range_time;
                            ob_pt_type5.x = tL_sig / (1 + exp(-ta_sig * (ob_pt_type5.y - tdy_sig))) + tdx_sig;
                            ob_pt_type5.z = 0.;
                        }

                        sampling_pt.x = 0.;
                        sampling_pt.y = 0.;
                        sampling_pt.z = 0.;
                        if (check_type1) {
                            sampling_pt.x += ob_pt_type1.x * (all_parameter.trajectry_weight[0] / sum_weight);
                            sampling_pt.y += ob_pt_type1.y * (all_parameter.trajectry_weight[0] / sum_weight);
                        }
                        if (check_type2) {
                            sampling_pt.x += ob_pt_type2.x * (all_parameter.trajectry_weight[1] / sum_weight);
                            sampling_pt.y += ob_pt_type2.y * (all_parameter.trajectry_weight[1] / sum_weight);
                        }
                        if (check_type3) {
                            sampling_pt.x += ob_pt_type3.x * (all_parameter.trajectry_weight[2] / sum_weight);
                            sampling_pt.y += ob_pt_type3.y * (all_parameter.trajectry_weight[2] / sum_weight);
                        }
                        if (check_type4) {
                            sampling_pt.x += ob_pt_type4.x * (all_parameter.trajectry_weight[3] / sum_weight);
                            sampling_pt.y += ob_pt_type4.y * (all_parameter.trajectry_weight[3] / sum_weight);
                        }
                        if (check_type5) {
                            sampling_pt.x += ob_pt_type5.x * (all_parameter.trajectry_weight[4] / sum_weight);
                            sampling_pt.y += ob_pt_type5.y * (all_parameter.trajectry_weight[4] / sum_weight);
                        }
                        if (dist_robot_ob < std::sqrt(std::pow(sampling_pt.x - robot_pt.x, 2.) + std::pow(sampling_pt.y - robot_pt.y, 2.))) points_steps.point_next.push_back(p3);  // 残す　＝　step=0がベストだったというだけ
                    }
                    else points_steps.point_next.push_back(next_pt);
                }
                // geometry_msgs::Point p1, p2, p3;
                // int mode = 0;
                // float a, b, c;
                // points_steps.point_next.clear();
                // for (int i=0; i<points_steps.point3.size(); i++)
                // {
                //     if ((i < points_steps.point1.size()) && (i < points_steps.point2.size()))
                //     {
                //         if ((std::isnan(points_steps.point1[i].x) != true) && (std::isnan(points_steps.point1[i].y) != true) && (std::isnan(points_steps.point2[i].x) != true) && (std::isnan(points_steps.point2[i].y) != true) && (std::isnan(points_steps.point3[i].x) != true) && (std::isnan(points_steps.point3[i].y) != true))
                //         {
                //             p1 = Pointtransform(all_parameter.map, all_parameter.robot_base, points_steps.point1[i]);
                //             p2 = Pointtransform(all_parameter.map, all_parameter.robot_base, points_steps.point2[i]);
                //             p3 = Pointtransform(all_parameter.map, all_parameter.robot_base, points_steps.point3[i]);
                //             MatrixXd A(3, 2);
                //             VectorXd B(3);
                //             A(0, 0) = p1.x;
                //             A(0, 1) = 1.0;
                //             B(0) = p1.y;
                //             A(1, 0) = p2.x;
                //             A(1, 1) = 1.0;
                //             B(1) = p2.y;
                //             A(2, 0) = p3.x;
                //             A(2, 1) = 1.0;
                //             B(2) = p3.y;
                //             Vector2d result = A.colPivHouseholderQr().solve(B);
                //             float d1 = std::fabs(result[0]*p1.x - 1*p1.y + result[1]) / sqrtf(powf(result[0], 2.) + 1.);
                //             float d2 = std::fabs(result[0]*p2.x - 1*p2.y + result[1]) / sqrtf(powf(result[0], 2.) + 1.);
                //             float d3 = std::fabs(result[0]*p3.x - 1*p3.y + result[1]) / sqrtf(powf(result[0], 2.) + 1.);
                //             if ((d1 <= all_parameter.human_noise/2.) && (d2 <= all_parameter.human_noise/2.) && (d3 <= all_parameter.human_noise/2.))
                //             {
                //                 mode = 0;
                //             }
                //             else if ((p1.x != p2.x) && (p2.x != p3.x) && (p3.x != p1.x) && ((p1.y != p2.y) && (p2.y != p3.y) && (p3.y != p1.y)))
                //             {
                //                 if ((((p1.x < p2.x) && (p2.x < p3.x)) || ((p1.x > p2.x) && (p2.x > p3.x))) && (((p1.y < p2.y) && (p2.y < p3.y)) || ((p1.y > p2.y) && (p2.y > p3.y))))
                //                 {
                //                     float a_base_x, b_base_x, c_base_x, a_base_y, b_base_y, c_base_y;
                //                     a_base_x = ((p1.y - p2.y) / ((p1.x - p2.x) * (p2.x - p3.x))) - ((p1.y - p3.y) / ((p1.x - p3.x) * (p2.x - p3.x)));
                //                     b_base_x = (p1.y - p2.y) / (p1.x - p2.x) - a_base_x * (p1.x + p2.x);
                //                     c_base_x = p1.y - a_base_x * powf(p1.x, 2.) - b_base_x * p1.x;
                //                     a_base_y = ((p1.x - p2.x) / ((p1.y - p2.y) * (p2.y - p3.y))) - ((p1.x - p3.x) / ((p1.y - p3.y) * (p2.y - p3.y)));
                //                     b_base_y = (p1.x - p2.x) / (p1.y - p2.y) - a_base_y * (p1.y + p2.y);
                //                     c_base_y = p1.x - a_base_y * powf(p1.y, 2.) - b_base_y * p1.y;
                //                     if ((((p1.x < p2.x) && (p2.x < p3.x) && (p3.x < ((-1)*(b_base_x / (2*a_base_x))))) || ((p1.x > p2.x) && (p2.x > p3.x) && (p3.x > ((-1)*(b_base_x / (2*a_base_x)))))) && 
                //                         (((p1.y < p2.y) && (p2.y < p3.y) && (p3.y < ((-1)*(b_base_y / (2*a_base_y))))) || ((p1.y > p2.y) && (p2.y > p3.y) && (p3.y > ((-1)*(b_base_y / (2*a_base_y)))))))
                //                     {
                //                         mode = 0;
                //                     }
                //                     else if (((p1.x < p2.x) && (p2.x < p3.x) && (p3.x < ((-1)*(b_base_x / (2*a_base_x))))) || ((p1.x > p2.x) && (p2.x > p3.x) && (p3.x > ((-1)*(b_base_x / (2*a_base_x))))))
                //                     {
                //                         mode = 1;
                //                     }
                //                     else if (((p1.y < p2.y) && (p2.y < p3.y) && (p3.y < ((-1)*(b_base_y / (2*a_base_y))))) || ((p1.y > p2.y) && (p2.y > p3.y) && (p3.y > ((-1)*(b_base_y / (2*a_base_y))))))
                //                     {
                //                         mode = 2;
                //                     }
                //                     else
                //                     {
                //                         mode = 0;
                //                     }
                //                 }
                //                 else if (((p1.x < p2.x) && (p2.x < p3.x)) || ((p1.x > p2.x) && (p2.x > p3.x)))
                //                 {
                //                     mode = 1;
                //                 }
                //                 else if (((p1.y < p2.y) && (p2.y < p3.y)) || ((p1.y > p2.y) && (p2.y > p3.y)))
                //                 {
                //                     mode = 2;
                //                 }
                //                 else
                //                 {
                //                     mode = 0;
                //                 }
                //             }
                //             if (((p1.x != p2.x) && (p2.x != p3.x) && (p3.x != p1.x)) && ((p1.y == p2.y) || (p2.y == p3.y) || (p3.x == p1.y)))
                //             {
                //                 mode = 1;
                //             }
                //             else if (((p1.y != p2.y) && (p2.y != p3.y) && (p3.x != p1.y)) && ((p1.x == p2.x) || (p2.x == p3.x) || (p3.x == p1.x)))
                //             {
                //                 mode = 2;
                //             }
                //             else
                //             {
                //                 mode = 0;
                //             }
                //             if ((all_parameter.min_human_vel <= sqrtf(powf((p3.x - p1.x)/2, 2) + powf((p3.y - p1.y)/2, 2))/all_parameter.detect_range_time) && (sqrtf(powf((p3.x - p1.x)/2, 2) + powf((p3.y - p1.y)/2, 2))/all_parameter.detect_range_time <= all_parameter.max_human_vel))
                //             {
                //                 float angle = atan(result[0]);
                //                 if ((std::fabs(angle - atan2(p3.y - p2.y , p3.x - p2.x) - 2*M_PI) >= (M_PI/2.)) && (std::fabs(angle - atan2(p3.y - p2.y , p3.x - p2.x)) >= (M_PI/2.)) && (std::fabs(angle - atan2(p3.y - p2.y , p3.x - p2.x) + 2*M_PI) >= (M_PI/2.)))
                //                 {
                //                     angle -= M_PI*(angle/std::fabs(angle));
                //                 }
                //                 if ((std::fabs(angle) <= M_PI/4) || (3*M_PI/4 <= std::fabs(angle)))
                //                 {
                //                     ob_point_temp.x = p3.x - 1.3 * std::fabs(p3.x) * (((p3.x - p2.x) + (p2.x - p1.x)) / (2*all_parameter.detect_range_time)) / ((((p3.x - p2.x) + (p2.x - p1.x)) / (2*all_parameter.detect_range_time)) - sqrtf(pow(robot_vel.linear.x, 2.) + pow(robot_vel.linear.y, 2.)));
                //                     ob_point_temp.y = result[0] * ob_point_temp.x + result[1];
                //                     if (ob_point_temp.x <= all_parameter.lidar_pose[0] + 0.1)
                //                     {
                //                         ob_point_temp.x = all_parameter.lidar_pose[0] + 0.1;
                //                         ob_point_temp.y = result[0] * ob_point_temp.x + result[1];
                //                     }
                //                 }
                //                 else
                //                 {
                //                     ob_point_temp.y = p3.y - std::fabs(p3.y) * (((p3.y - p2.y) + (p2.y - p1.y)) / (2*all_parameter.detect_range_time)) / ((((p3.y - p2.y) + (p2.y - p1.y)) / (2*all_parameter.detect_range_time)) - sqrtf(pow(robot_vel.linear.x, 2.) + pow(robot_vel.linear.y, 2.)));
                //                     ob_point_temp.x = (ob_point_temp.y - result[1]) / result[0];
                //                 }
                //                 points_steps.point_next.push_back(ob_point_temp);
                //             }
                //             else
                //             {
                //                 ob_point_temp.x = p3.x;
                //                 ob_point_temp.y = p3.y;
                //                 points_steps.point_next.push_back(ob_point_temp);
                //             }
                //         }
                //     }
                // }
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