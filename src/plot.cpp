#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <string>
#include <iostream>
#include <limits>
#include <cmath>
#include <math.h>
#include <typeinfo>
#include <sys/time.h>
#include <matplotlib-cpp/matplotlibcpp.h>
#include <navigation_stack/MapInformation.h>
#include <navigation_stack/ExpansionPoints.h>
#include <navigation_stack/PathPoint.h>
#include <visualization_msgs/MarkerArray.h>



class GRIDDING
{
    public:
        float size = 0.1;
        float arg_size = 1/size;
        float float_to_grid(float s, bool f=true)  // 適当な値をgrid幅に矯正する関数
        {
            float r = s - (((float)(s/size) - (int)(s/size))*size);
            if ((s<0) && (f))
            {
                r-=size;
            }
            r += (size/2);
            return r;
        }
        int float_to_int(float s, bool f=true)  // grid幅の値を0を基準にした格納番号(int型)に変換する関数
        {
            int r = s*arg_size;
            if ((s<0) && (f))
            {
                r--;
            }
            return r;
        }
        float int_to_grid(int s)  // float_to_intの逆をする
        {
            return (float)((s/arg_size) + (size/2.0));
        }
};


class ROBOT_POSITION
{
    private:
        ros::Subscriber sub_odom;
        ros::Subscriber sub_robot;
        bool f;
        void callback_odom(const nav_msgs::Odometry &odom)
        {
            odom_pose.x = odom.pose.pose.position.x;
            odom_pose.y = odom.pose.pose.position.y;
            odom_pose.z = odom.pose.pose.position.z;
            odom_theta = (2*(acos(odom.pose.pose.orientation.w)))*((odom.pose.pose.orientation.z)*(odom.pose.pose.orientation.w))/(std::fabs((odom.pose.pose.orientation.z)*(odom.pose.pose.orientation.w)));
            if (std::isnan(odom_theta) == true)
            {
                odom_theta = 0.0;
            }
            if ((std::fabs(odom_theta)) > M_PI)
            {
                odom_theta = (2*M_PI - std::fabs(odom_theta))*(((odom.pose.pose.orientation.z)*(odom.pose.pose.orientation.w))/(std::fabs((odom.pose.pose.orientation.z)*(odom.pose.pose.orientation.w))));
            }
            robot_pose.position.x = robot_pose_stack.position.x + odom_pose.x - odom_pose_stack.x;
            robot_pose.position.y = robot_pose_stack.position.y + odom_pose.y - odom_pose_stack.y;
            robot_pose.position.z = 0.03;
            robot_theta = robot_theta_stack + odom_theta - odom_theta_stack;
            robot_pose.orientation.w = cos(robot_theta / 2.);
            robot_pose.orientation.x = 0.;
            robot_pose.orientation.y = 0.;
            robot_pose.orientation.z = sin(robot_theta / 2.);
        }
        void callback_robot(const geometry_msgs::Pose &robot)
        {
            robot_pose_stack.position.x = robot.position.x;
            robot_pose_stack.position.y = robot.position.y;
            robot_pose_stack.position.z = robot.position.z + 0.03;
            robot_theta_stack = (2*(acos(robot.orientation.w)))*((robot.orientation.z)*(robot.orientation.w))/(std::fabs((robot.orientation.z)*(robot.orientation.w)));
            if (std::isnan(robot_theta_stack) == true)
            {
                robot_theta_stack = 0.0;
            }
            if ((std::fabs(robot_theta_stack)) > M_PI)
            {
                robot_theta_stack = (2*M_PI - std::fabs(robot_theta_stack))*(((robot.orientation.z)*(robot.orientation.w))/(std::fabs((robot.orientation.z)*(robot.orientation.w))));
            }
            robot_pose_stack.orientation.w = cos(robot_theta_stack / 2.);
            robot_pose_stack.orientation.x = 0.;
            robot_pose_stack.orientation.y = 0.;
            robot_pose_stack.orientation.z = sin(robot_theta_stack / 2.);
            odom_pose_stack.x = odom_pose.x;
            odom_pose_stack.y = odom_pose.y;
            odom_theta_stack = odom_theta;
            f = true;
        }
    public:
        geometry_msgs::Pose robot_pose;
        geometry_msgs::Pose robot_pose_stack;
        geometry_msgs::Point odom_pose;
        float robot_theta = 0.0;
        float robot_theta_stack = 0.0;
        float odom_theta = 0.0;
        geometry_msgs::Point odom_pose_stack;
        float odom_theta_stack = 0.0;
        ROBOT_POSITION()
        {
            ros::NodeHandle node;
            sub_odom = node.subscribe("/odom", 10, &ROBOT_POSITION::callback_odom, this);
            sub_robot = node.subscribe("/robot_position", 10, &ROBOT_POSITION::callback_robot, this);
            odom_pose.x = 0.;
            odom_pose.y = 0.;
            odom_pose.z = 0.;
            odom_pose_stack.x = 0.;
            odom_pose_stack.y = 0.;
            odom_pose_stack.z = 0.;
            get_point();
        }
        void get_point()
        {
            f = false;
            ros::spinOnce();
            while (ros::ok())
            {
                ros::spinOnce();
                if (f)
                {
                    odom_pose_stack.x = odom_pose.x;
                    odom_pose_stack.y = odom_pose.y;
                    odom_pose_stack.z = odom_pose.z;
                    odom_theta_stack = odom_theta;
                    break;
                }
            }
        }
};


class OBSTACLE_DIST
{
    private:
        ros::Subscriber sub_dist;
        geometry_msgs::Point point;
        float lidar_pose[2] = {0.0, 0.0};
        bool start_frag;
        void callback_obstacle(const sensor_msgs::LaserScan &ob)
        {
            // range.clear();
            ob_theta.clear();
            range_point.clear();
            range_angle_increment = ob.angle_increment;
            for (int i=0; i<ob.ranges.size(); i++)
            {
                if ((ob.range_min <= ob.ranges[i]) && (ob.ranges[i] <= ob.range_max))
                {
                    point.x = robot_position.robot_pose.position.x + (ob.ranges[i]*(cos(robot_position.robot_theta + ob.angle_min + range_angle_increment*i))) + (lidar_pose[0]*cos(robot_position.robot_theta) - lidar_pose[1]*sin(robot_position.robot_theta));
                    point.y = robot_position.robot_pose.position.y + (ob.ranges[i]*(sin(robot_position.robot_theta + ob.angle_min + range_angle_increment*i))) + (lidar_pose[0]*sin(robot_position.robot_theta) + lidar_pose[1]*cos(robot_position.robot_theta));
                    point.z = 0.02;
                    ob_theta.push_back(robot_position.robot_theta + ob.angle_min + range_angle_increment*i);
                    range_point.push_back(point);
                }
            }
            start_frag = true;
        }
    public:
        ROBOT_POSITION robot_position;
        std::vector<float> ob_theta;
        // std::vector<float> range;
        std::vector<geometry_msgs::Point> range_point;
        // std::vector<std::vector<double>> range_point;
        // long range_size = std::numeric_limits<int>::max();
        float range_angle_increment/*, range_max, range_min*/;
        OBSTACLE_DIST()
        {
            ros::NodeHandle node;
            sub_dist = node.subscribe("/scan", 10, &OBSTACLE_DIST::callback_obstacle, this);
            get_dist();
        }
        void get_dist()
        {
            start_frag = false;
            ros::spinOnce();
            while(ros::ok())
            {
                ros::spinOnce();
                if (start_frag)
                {
                    ros::spinOnce();
                    break;
                }
            }
            ros::spinOnce();
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
        ros::Publisher pub_marker_map;
        ros::Publisher pub_marker_robot;
        ros::Publisher pub_marker_ob;
        ros::Publisher pub_marker_expansion;
        ros::Publisher pub_marker_globalpath;
        ros::Subscriber sub_map;
        ros::Subscriber sub_marker_expansion;
        ros::Subscriber sub_marker_globalpath;
        RvizMarkerLibrary marker_lib;
        ROBOT_POSITION robot_position;
        OBSTACLE_DIST obstacle_dist;
        navigation_stack::MapInformation plot_map;
        std::vector<geometry_msgs::Point> expansion_pose;
        std::vector<geometry_msgs::Point> globalpath_pose;
        bool map_set_frag,expansion_frag,globalpath_frag;
        void callback_map(const navigation_stack::MapInformation &get_map)
        {
            plot_map.cost.clear();
            plot_map.clearly.clear();
            for (int i=0; i<get_map.cost.size(); i++)
            {
                plot_map.cost.push_back(get_map.cost[i]);
            }
            for (int i=0; i<get_map.clearly.size(); i++)
            {
                plot_map.clearly.push_back(get_map.clearly[i]);
            }
            map_set_frag = true;
        }
        void callback_expansion(const navigation_stack::ExpansionPoints &get_ex)
        {
            geometry_msgs::Point pt_e;
            pt_e.z = 0.00;
            expansion_pose.clear();
            for (int i=0; i<get_ex.poses.size(); i++)
            {
                pt_e.x = get_ex.poses[i].x;
                pt_e.y = get_ex.poses[i].y;
                expansion_pose.push_back(pt_e);
            }
            expansion_frag = true;
        }
        void callback_globalpath(const navigation_stack::PathPoint &gpp)
        {
            geometry_msgs::Point pt_e;
            pt_e.z = 0.00;
            globalpath_pose.clear();
            for (int i=0; i<gpp.poses.size(); i++)
            {
                pt_e.x = gpp.poses[i].x;
                pt_e.y = gpp.poses[i].y;
                globalpath_pose.push_back(pt_e);
            }
            globalpath_frag = true;
        }
    public:
        PLOT()
        {
            ros::NodeHandle node;
            pub_marker_map = node.advertise<visualization_msgs::Marker>("/active_map", 10);
            pub_marker_robot = node.advertise<visualization_msgs::Marker>("/active_map", 10);
            pub_marker_ob = node.advertise<visualization_msgs::Marker>("/active_map", 10);
            pub_marker_expansion = node.advertise<visualization_msgs::Marker>("/active_map", 10);
            pub_marker_globalpath = node.advertise<visualization_msgs::Marker>("/active_map", 10);
            sub_map = node.subscribe("/mapping", 10, &PLOT::callback_map, this);
            sub_marker_expansion = node.subscribe("/expansion_poses", 10, &PLOT::callback_expansion, this);
            sub_marker_globalpath = node.subscribe("/global_path_planning", 10, &PLOT::callback_globalpath, this);
            get_plot();
            mapping_plot();
        }
        void get_plot()
        {
            map_set_frag = false;
            expansion_frag = false;
            globalpath_frag = false;
            while (ros::ok())
            {
                if (map_set_frag)
                {
                    break;
                }
                ros::spinOnce();
            }
        }
        void mapping_plot()
        {
            GRIDDING gridding;
            std::vector<geometry_msgs::Point> points;
            std::vector<std_msgs::ColorRGBA> colors;
            geometry_msgs::Vector3 scale;
            geometry_msgs::Vector3 local_ob_size;
            geometry_msgs::Vector3 robot_size;
            geometry_msgs::Vector3 grid_size;
            geometry_msgs::Pose pose;
            geometry_msgs::Pose robot_pose_plot;
            scale.x = gridding.size*1.0;
            scale.y = gridding.size*1.0;
            scale.z = 0.00;
            local_ob_size.x = gridding.size/3;
            local_ob_size.y = gridding.size/3;
            local_ob_size.z = 0.0;
            grid_size.x = gridding.size;
            grid_size.y = gridding.size;
            grid_size.z = 0.01;
            robot_size.x = 0.5;
            robot_size.y = 0.15;
            robot_size.z = 0.00;
            pose.position.x = 0.0;
            pose.position.y = 0.0;
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            geometry_msgs::Point pt;
            std_msgs::ColorRGBA rgb;
            std_msgs::ColorRGBA robot_color;
            std::vector<std_msgs::ColorRGBA> ob_colors;
            std::vector<std_msgs::ColorRGBA> expansion_colors;
            std::vector<std_msgs::ColorRGBA> globalpath_colors;
            robot_color.a = 1.0;
            robot_color.r = 0.0;
            robot_color.g = 0.0;
            robot_color.b = 1.0;
            pt.z = 0;
            rgb.a = 1.0;
            while (ros::ok())
            {
                points.clear();
                colors.clear();
                ob_colors.clear();
                expansion_colors.clear();
                globalpath_colors.clear();
                for (int i=0; i<plot_map.cost.size(); i++)
                {
                    rgb.r = 0;
                    rgb.g = 0;
                    rgb.b = 0;
                    pt.x = plot_map.cost[i].x;
                    pt.y = plot_map.cost[i].y;
                    points.push_back(pt);
                    colors.push_back(rgb);
                }
                for (int i=0; i<plot_map.clearly.size(); i++)
                {
                    rgb.r = 1;
                    rgb.g = 1;
                    rgb.b = 1;
                    pt.x = plot_map.clearly[i].x;
                    pt.y = plot_map.clearly[i].y;
                    points.push_back(pt);
                    colors.push_back(rgb);
                }
                ros::spinOnce();
                pub_marker_map.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::CUBE_LIST, "map", "cube_list1", pose, points, scale, colors, ros::Duration(1) ) );
                // obstacle_dist.robot_position.robot_pose.position.x += obstacle_dist.robot_position.odom_pose.x - obstacle_dist.robot_position.odom_pose_stack.x;
                // obstacle_dist.robot_position.robot_pose.position.y += obstacle_dist.robot_position.odom_pose.y - obstacle_dist.robot_position.odom_pose_stack.y;
                // obstacle_dist.robot_position.robot_pose.position.z = 0.03;
                // obstacle_dist.robot_position.robot_theta += obstacle_dist.robot_position.odom_theta - obstacle_dist.robot_position.odom_theta_stack;
                // while (ros::ok())
                // {
                //     if (std::fabs(obstacle_dist.robot_position.robot_theta) > M_PI)
                //     {
                //         if (obstacle_dist.robot_position.robot_theta > 0.)
                //         {
                //             obstacle_dist.robot_position.robot_theta -= 2*M_PI;
                //         }
                //         else
                //         {
                //             obstacle_dist.robot_position.robot_theta += 2*M_PI;
                //         }
                //     }
                //     if (std::fabs(obstacle_dist.robot_position.robot_theta) <= M_PI)
                //     {
                //         break;
                //     }
                // }
                // obstacle_dist.robot_position.robot_pose.orientation.w = cos(obstacle_dist.robot_position.robot_theta / 2.);
                // obstacle_dist.robot_position.robot_pose.orientation.x = 0.;
                // obstacle_dist.robot_position.robot_pose.orientation.y = 0.;
                // obstacle_dist.robot_position.robot_pose.orientation.z = sin(obstacle_dist.robot_position.robot_theta / 2.);
                pub_marker_robot.publish( marker_lib.makeMarker( visualization_msgs::Marker::ARROW, "map", "arrow", robot_position.robot_pose, robot_size, robot_color, ros::Duration(1) ) );
                // obstacle_dist.robot_position.odom_pose_stack.x = obstacle_dist.robot_position.odom_pose.x;
                // obstacle_dist.robot_position.odom_pose_stack.y = obstacle_dist.robot_position.odom_pose.y;
                // obstacle_dist.robot_position.odom_pose_stack.z = obstacle_dist.robot_position.odom_pose.z;
                // obstacle_dist.robot_position.odom_theta_stack = obstacle_dist.robot_position.odom_theta;
                rgb.r = 1.0;
                rgb.g = 0.0;
                rgb.b = 0.0;
                ob_colors.resize(obstacle_dist.range_point.size(),rgb);
                pub_marker_ob.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::POINTS, "map", "point_list1", pose, obstacle_dist.range_point, local_ob_size, ob_colors, ros::Duration(1) ) );
                if (expansion_frag)
                {
                    rgb.r = 1.0;
                    rgb.g = 1.0;
                    rgb.b = 0.0;
                    expansion_colors.resize(expansion_pose.size(),rgb);
                    pub_marker_expansion.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::CUBE_LIST, "map", "cube_list2", pose, expansion_pose, grid_size, expansion_colors, ros::Duration(1) ) );
                }
                if (globalpath_frag)
                {
                    rgb.r = 0.0;
                    rgb.g = 1.0;
                    rgb.b = 0.0;
                    globalpath_colors.resize(globalpath_pose.size(),rgb);
                    pub_marker_globalpath.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::CUBE_LIST, "map", "cube_list2", pose, globalpath_pose, grid_size, globalpath_colors, ros::Duration(1) ) );
                }
                ros::spinOnce();
            }
        }
};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_plot");
    PLOT plot;
    ros::spin();
    return 0;
}