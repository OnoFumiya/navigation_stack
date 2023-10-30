#include <stdio.h>
#include <ros/ros.h>
#include <dr_spaam_ros/LegPoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <limits>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Geometry>
#include <navigation_stack/WalkLegPoint.h>


class RvizMarkerLibrary
{
    public:
        RvizMarkerLibrary()
        {}
        visualization_msgs::Marker makeMarker(const int type, const std::string& frame_id, const std::string& name, const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& scale, const std_msgs::ColorRGBA& color, const ros::Duration& lifetime)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = name;
            marker.id = 0;
            marker.type = type;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale = scale;
            marker.color = color;
            marker.lifetime = lifetime;
            marker.pose = pose;
            return marker;
        }
        visualization_msgs::Marker makeMarkerList(const int type, const std::string& frame_id, const std::string& name, const geometry_msgs::Pose& pose, const std::vector<geometry_msgs::Point>& points, const geometry_msgs::Vector3& scale, const std::vector<std_msgs::ColorRGBA>& colors, const ros::Duration& lifetime)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = name;
            marker.id = 0;
            marker.type = type;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale = scale;
            marker.points = points;
            marker.colors = colors;
            marker.lifetime = lifetime;
            marker.pose = pose;
            return marker;
        }
};



class PLOT
{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub_marker_lidar;
        ros::Publisher pub_marker_leg1;
        ros::Publisher pub_marker_leg2;
        ros::Publisher pub_marker_leg3;
        ros::Publisher pub_marker_leg_next;
        ros::Publisher pub_leg_cost;
        ros::Subscriber removal_scan_sub;
        ros::Subscriber legs_sub;
        RvizMarkerLibrary marker_lib;
        float lidar_pose[2] = {0.2, 0.0};
        float detect_range_time = 0.1;
        bool start_flag_leg, start_flag_scan;
        std_msgs::ColorRGBA red_color;
        std_msgs::ColorRGBA white_color;
        std_msgs::ColorRGBA green_color;
        std_msgs::ColorRGBA blue_color;
        std_msgs::ColorRGBA yellow_color;
        std_msgs::ColorRGBA orange_color;
        std::vector<std_msgs::ColorRGBA> cube_colors;
        geometry_msgs::Vector3 point_scale;
        geometry_msgs::Vector3 cylinder_scale;
        geometry_msgs::Vector3 arrow_scale;
        geometry_msgs::Pose defalt_pose;
        std::vector<geometry_msgs::Point> leg_points_temp;
        // std::vector<std::vector<geometry_msgs::Point>> leg_points_samplings;
        navigation_stack::WalkLegPoint leg_points_steps;
        std::vector<std::vector<int>> index_combination;
        // int combination[sampling_step];
        sensor_msgs::PointCloud pcl;
        Eigen::Quaterniond thetaToQuaternion(double theta)
        {
            // 角度と軸ベクトルを指定してAngleAxisを作成
            Eigen::AngleAxisd angleAxis(theta, Eigen::Vector3d::UnitZ());
            
            // AngleAxisを4元数に変換
            Eigen::Quaterniond quaternion(angleAxis);
            
            return quaternion;
        }
        void callback_scan(const sensor_msgs::LaserScan &msg)
        {
            lidar_points.clear();
            geometry_msgs::Point point;
            float a;
            float range_angle_increment = msg.angle_increment;
            for (int i=0; i<msg.ranges.size(); i++)
            {
                if ((msg.range_min <= msg.ranges[i]) && (msg.ranges[i] <= msg.range_max))
                {
                    a = (msg.angle_min + range_angle_increment*i);
                    point.x = msg.ranges[i]*(cos(a)) + lidar_pose[0];
                    point.y = msg.ranges[i]*(sin(a)) + lidar_pose[1];
                    point.z = 0.02;
                    lidar_points.push_back(point);
                }
            }
            start_flag_scan = true;
        }
        void callback_legs(const navigation_stack::WalkLegPoint &wlp)
        {
            leg_points_steps.point1.resize(wlp.point1.size());
            leg_points_steps.point2.resize(wlp.point2.size());
            leg_points_steps.point3.resize(wlp.point3.size());
            leg_points_steps.point_next.resize(wlp.point_next.size());
            copy(wlp.point1.begin(), wlp.point1.end(), leg_points_steps.point1.begin());
            copy(wlp.point2.begin(), wlp.point2.end(), leg_points_steps.point2.begin());
            copy(wlp.point3.begin(), wlp.point3.end(), leg_points_steps.point3.begin());
            copy(wlp.point_next.begin(), wlp.point_next.end(), leg_points_steps.point_next.begin());
            pcl.header.stamp = ros::Time::now();
            pcl.header.frame_id = "base_footprint";
            pcl.points.clear();
            for (int i=0; i<leg_points_steps.point_next.size(); i++)
            {
                geometry_msgs::Point32 pt;
                pt.x = leg_points_steps.point_next[i].x;
                pt.y = leg_points_steps.point_next[i].y;
                pt.z = leg_points_steps.point_next[i].z;
                pcl.points.push_back(pt);
            }
            start_flag_leg = true;
        }
    public:
        std::vector<geometry_msgs::Point> lidar_points;
        PLOT()
        {
            removal_scan_sub = nh.subscribe("/dr_spaam_navigation/scan", 10, &PLOT::callback_scan, this);
            legs_sub = nh.subscribe("/dr_spaam_navigation/leg_pointers_step", 1, &PLOT::callback_legs, this);
            start_flag_leg = false;
            start_flag_scan = false;
            while (ros::ok())
            {
                ros::spinOnce();
                if ((start_flag_scan) && (start_flag_leg))
                {
                    ros::spinOnce();
                    break;
                }
                ros::spinOnce();
            }
            ros::spinOnce();
            pub_marker_lidar = nh.advertise<visualization_msgs::Marker>("/dr_spaam_navigation/lidar_points", 10);
            pub_marker_leg1 = nh.advertise<visualization_msgs::Marker>("/dr_spaam_navigation/leg_point_step1", 10);
            pub_marker_leg2 = nh.advertise<visualization_msgs::Marker>("/dr_spaam_navigation/leg_point_step2", 10);
            pub_marker_leg3 = nh.advertise<visualization_msgs::Marker>("/dr_spaam_navigation/leg_point_step3", 10);
            pub_marker_leg_next = nh.advertise<visualization_msgs::Marker>("/dr_spaam_navigation/leg_point_nextstep", 10);
            pub_leg_cost = nh.advertise<sensor_msgs::PointCloud>("/dr_spaam_navigation/leg_cost", 10);
            defalt_pose.position.x = 0.0;
            defalt_pose.position.y = 0.0;
            defalt_pose.position.z = 0.0;
            defalt_pose.orientation.w = 1.0;
            defalt_pose.orientation.x = 0.0;
            defalt_pose.orientation.y = 0.0;
            defalt_pose.orientation.z = 0.0;
            point_scale.x = 0.03;
            point_scale.y = 0.03;
            point_scale.z = 0.00;
            cylinder_scale.x = 0.15;
            cylinder_scale.y = 0.15;
            cylinder_scale.z = 0.15;
            arrow_scale.x = 0.2;
            arrow_scale.y = 0.2;
            arrow_scale.z = 0.2;
            red_color.r = 1.0;
            red_color.g = 0.0;
            red_color.b = 0.0;
            red_color.a = 1.0;
            white_color.r = 1.0;
            white_color.g = 1.0;
            white_color.b = 1.0;
            white_color.a = 1.0;
            green_color.r = 0.0;
            green_color.g = 1.0;
            green_color.b = 0.0;
            green_color.a = 1.0;
            blue_color.r = 0.0;
            blue_color.g = 0.0;
            blue_color.b = 1.0;
            blue_color.a = 1.0;
            yellow_color.r = 1.0;
            yellow_color.g = 1.0;
            yellow_color.b = 0.0;
            yellow_color.a = 1.0;
            orange_color.r = 1.0;
            orange_color.g = 0.5;
            orange_color.b = 0.0;
            orange_color.a = 1.0;
            ploter();
        }
        void ploter()
        {
            while (ros::ok())
            {
                cube_colors.assign(lidar_points.size(),red_color);
                pub_marker_lidar.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::POINTS, "base_footprint", "lidar_point", defalt_pose, lidar_points, point_scale, cube_colors, ros::Duration(1) ) );
                cube_colors.assign(leg_points_steps.point1.size(),white_color);
                pub_marker_leg1.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::SPHERE_LIST, "base_footprint", "point_list1", defalt_pose, leg_points_steps.point1, cylinder_scale, cube_colors, ros::Duration(1) ) );
                cube_colors.assign(leg_points_steps.point2.size(),blue_color);
                pub_marker_leg2.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::SPHERE_LIST, "base_footprint", "point_list2", defalt_pose, leg_points_steps.point2, cylinder_scale, cube_colors, ros::Duration(1) ) );
                cube_colors.assign(leg_points_steps.point3.size(),green_color);
                pub_marker_leg3.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::SPHERE_LIST, "base_footprint", "point_list3", defalt_pose, leg_points_steps.point3, cylinder_scale, cube_colors, ros::Duration(1) ) );
                cube_colors.assign(leg_points_steps.point_next.size(),yellow_color);
                pub_marker_leg_next.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::SPHERE_LIST, "base_footprint", "point_next", defalt_pose, leg_points_steps.point_next, cylinder_scale, cube_colors, ros::Duration(1) ) );
                pub_leg_cost.publish(pcl);
                ros::spinOnce();
            }
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "leg_plot");
    PLOT plot;
    ros::spin();
    return 0;
}