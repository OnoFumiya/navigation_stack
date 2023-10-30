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
#include <limits>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Geometry>
#include <navigation_stack/WalkLegPoint.h>
#include <tf/transform_listener.h>


class HUMAN_DETECT
{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub_walk_leg;
        ros::Publisher pub_legremoval_scan;
        tf::TransformListener tf_listener;
        ros::Subscriber leg_sub;
        float lidar_pose[2] = {0.2, 0.0};
        int sampling_step = 5;
        float max_human_vel = 1.5;
        float max_human_radius = 0.80;
        float human_noise = 0.4;
        float detect_range_time = 0.1;
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
                point.x = msg.poses[i].position.x + lidar_pose[0];
                point.y = msg.poses[i].position.y + lidar_pose[1];
                point.z = 0.2;
                if ((msg.scan.range_min <= sqrtf(powf(point.x, 2.) + powf(point.y, 2.))) && (sqrtf(powf(point.x, 2.) + powf(point.y, 2.)) <= msg.scan.range_max))
                {
                    leg_points_base.push_back(point);
                    leg_points.push_back(Pointtransform("base_footprint", "map", point));
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
                        if (sqrtf(powf((msg.scan.ranges[i]*(cos(msg.scan.angle_min + range_angle_increment*i)) + lidar_pose[0]) - leg_points_base[j].x, 2.) + powf((msg.scan.ranges[i]*(sin(msg.scan.angle_min + range_angle_increment*i)) + lidar_pose[1]) - leg_points_base[j].y, 2.)) <= max_human_radius)
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
            pub_walk_leg = nh.advertise<navigation_stack::WalkLegPoint>("/dr_spaam_navigation/leg_pointers_step", 10);
            pub_legremoval_scan = nh.advertise<sensor_msgs::LaserScan>("/dr_spaam_navigation/scan", 10);
            leg_sub = nh.subscribe("/dr_spaam_detections", 10, &HUMAN_DETECT::callback_leg, this);
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
                for (int i=1; i<sampling_step; i++)
                {
                    ros::Duration(detect_range_time).sleep();
                    leg_points_temp.resize(leg_points.size());
                    copy(leg_points.begin(), leg_points.end(), leg_points_temp.begin());
                    for (int j=0; j<leg_points_temp.size(); j++)
                    {
                        bool input_flag = false;
                        for (int k=0; k<leg_points_samplings.size(); k++)
                        {
                            if (sqrtf(powf(leg_points_temp[j].x - leg_points_samplings[k][leg_points_samplings[k].size()-1].x, 2.) + powf(leg_points_temp[j].y - leg_points_samplings[k][leg_points_samplings[k].size()-1].y, 2.)) <= human_noise)
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
                        if (sqrtf(powf(leg_point_temp.x - leg_points_steps.point2[j].x, 2.) + powf(leg_point_temp.y - leg_points_steps.point2[j].y, 2.)) <= max_human_vel*detect_range_time*sampling_step)
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
                leg_points_steps.point_next.clear();
                for (int i=0; i<leg_points_steps.point3.size(); i++)
                {
                    if ((i < leg_points_steps.point1.size()) && (i < leg_points_steps.point2.size()))
                    {
                        if ((std::isnan(leg_points_steps.point1[i].x) != true) && (std::isnan(leg_points_steps.point1[i].y) != true) && (std::isnan(leg_points_steps.point2[i].x) != true) && (std::isnan(leg_points_steps.point2[i].y) != true) && (std::isnan(leg_points_steps.point3[i].x) != true) && (std::isnan(leg_points_steps.point3[i].y) != true))
                        {
                            leg_point_temp.x = leg_points_steps.point3[i].x + (leg_points_steps.point3[i].x - leg_points_steps.point2[i].x);
                            leg_point_temp.y = leg_points_steps.point3[i].y + (leg_points_steps.point3[i].y - leg_points_steps.point2[i].y);
                            leg_point_temp.z = 0.4;
                            leg_points_steps.point_next.push_back(leg_point_temp);
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