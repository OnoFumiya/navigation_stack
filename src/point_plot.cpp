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
        ros::Publisher pub_marker_step1;
        ros::Publisher pub_marker_step2;
        ros::Publisher pub_marker_step3;
        ros::Publisher pub_marker_step_next;
        ros::Publisher pub_cost;
        ros::Subscriber removal_scan_sub;
        ros::Subscriber points_sub;
        RvizMarkerLibrary marker_lib;
        ALL_PARAMETER all_parameter;
        // float lidar_pose[2] = {0.2, 0.0};
        // float detect_range_time = 0.1;
        bool start_flag_point, start_flag_scan;
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
        navigation_stack::WalkLegPoint object_points_steps;
        std::vector<std::vector<int>> index_combination;
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
                    point.x = msg.ranges[i]*(cos(a)) + all_parameter.lidar_pose[0];
                    point.y = msg.ranges[i]*(sin(a)) + all_parameter.lidar_pose[1];
                    point.z = 0.02;
                    lidar_points.push_back(point);
                }
            }
            start_flag_scan = true;
        }
        void callback_points(const navigation_stack::WalkLegPoint &wlp)
        {
            object_points_steps.point1.resize(wlp.point1.size());
            object_points_steps.point2.resize(wlp.point2.size());
            object_points_steps.point3.resize(wlp.point3.size());
            object_points_steps.point_next.resize(wlp.point_next.size());
            copy(wlp.point1.begin(), wlp.point1.end(), object_points_steps.point1.begin());
            copy(wlp.point2.begin(), wlp.point2.end(), object_points_steps.point2.begin());
            copy(wlp.point3.begin(), wlp.point3.end(), object_points_steps.point3.begin());
            copy(wlp.point_next.begin(), wlp.point_next.end(), object_points_steps.point_next.begin());
            pcl.header.stamp = ros::Time::now();
            pcl.header.frame_id = all_parameter.robot_base;
            pcl.points.clear();
            for (int i=0; i<object_points_steps.point_next.size(); i++)
            {
                geometry_msgs::Point32 pt;
                pt.x = object_points_steps.point_next[i].x;
                pt.y = object_points_steps.point_next[i].y;
                pt.z = 0.3;
                // pt.z = object_points_steps.point_next[i].z;
                pcl.points.push_back(pt);
            }
            start_flag_point = true;
        }
    public:
        std::vector<geometry_msgs::Point> lidar_points;
        PLOT()
        {
            removal_scan_sub = nh.subscribe(all_parameter.removal_scan_topic, 10, &PLOT::callback_scan, this);
            points_sub = nh.subscribe(all_parameter.stepping_point_topic, 1, &PLOT::callback_points, this);
            start_flag_point = false;
            start_flag_scan = false;
            while (ros::ok())
            {
                ros::spinOnce();
                if ((start_flag_scan) && (start_flag_point))
                {
                    ros::spinOnce();
                    break;
                }
                ros::spinOnce();
            }
            ros::spinOnce();
            pub_marker_lidar = nh.advertise<visualization_msgs::Marker>(all_parameter.removal_scan_marker, 10);
            pub_marker_step1 = nh.advertise<visualization_msgs::Marker>(all_parameter.step1_marker, 10);
            pub_marker_step2 = nh.advertise<visualization_msgs::Marker>(all_parameter.step2_marker, 10);
            pub_marker_step3 = nh.advertise<visualization_msgs::Marker>(all_parameter.step3_marker, 10);
            pub_marker_step_next = nh.advertise<visualization_msgs::Marker>(all_parameter.next_step_marker, 10);
            pub_cost = nh.advertise<sensor_msgs::PointCloud>(all_parameter.cost_topic, 1);
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
                pub_marker_lidar.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::POINTS, all_parameter.robot_base, "lidar_point", defalt_pose, lidar_points, point_scale, cube_colors, ros::Duration(1) ) );
                if (object_points_steps.point1.size() > 0)
                {
                    cube_colors.assign(object_points_steps.point1.size(),white_color);
                    pub_marker_step1.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::SPHERE_LIST, all_parameter.map, "point_list1", defalt_pose, object_points_steps.point1, cylinder_scale, cube_colors, ros::Duration(1) ) );
                }
                if (object_points_steps.point2.size() > 0)
                {
                    cube_colors.assign(object_points_steps.point2.size(),blue_color);
                    pub_marker_step2.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::SPHERE_LIST, all_parameter.map, "point_list2", defalt_pose, object_points_steps.point2, cylinder_scale, cube_colors, ros::Duration(1) ) );
                }
                if (object_points_steps.point3.size() > 0)
                {
                    cube_colors.assign(object_points_steps.point3.size(),green_color);
                    pub_marker_step3.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::SPHERE_LIST, all_parameter.map, "point_list3", defalt_pose, object_points_steps.point3, cylinder_scale, cube_colors, ros::Duration(1) ) );
                }
                if (object_points_steps.point_next.size() > 0)
                {
                    cube_colors.assign(object_points_steps.point_next.size(),yellow_color);
                    pub_marker_step_next.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::SPHERE_LIST, all_parameter.robot_base, "point_next", defalt_pose, object_points_steps.point_next, cylinder_scale, cube_colors, ros::Duration(1) ) );
                }
                pub_cost.publish(pcl);
                ros::spinOnce();
            }
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_plot");
    PLOT plot;
    ros::spin();
    return 0;
}