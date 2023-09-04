#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
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
#include <visualization_msgs/MarkerArray.h>



class GRIDDING
{
    public:
        float size = 0.1;
        float arg_size = 1/size;
        float float_to_grid(float s, bool f=true)
        {
            float r = s - (((float)(s/size) - (int)(s/size))*size);
            if ((s<0) && (f))
            {
                r-=size;
            }
            r += (size/2);
            return r;
        }
        int float_to_int(float s, bool f=true)
        {
            int r = s*arg_size;
            if ((s<0) && (f))
            {
                r--;
            }
            return r;
        }
        float int_to_grid(int s)
        {
            return (float)((s/arg_size) + (1/(2*arg_size)));
        }
};


class ROBOT_POSITION
{
    private:
        ros::Subscriber sub_odom;
        bool f;
        void callback_odom(const nav_msgs::Odometry &odom)
        {
            robot_pose.position.x = odom.pose.pose.position.x + missed.position.x;
            robot_pose.position.y = odom.pose.pose.position.y + missed.position.y;
            robot_pose.position.z = odom.pose.pose.position.z + missed.position.z + 0.03;
            robot_pose.orientation.w = odom.pose.pose.orientation.w + missed.orientation.w;
            robot_pose.orientation.x = odom.pose.pose.orientation.x + missed.orientation.x;
            robot_pose.orientation.y = odom.pose.pose.orientation.y + missed.orientation.y;
            robot_pose.orientation.z = odom.pose.pose.orientation.z + missed.orientation.z;
            sita = (2*(acos(odom.pose.pose.orientation.w + missed.orientation.w)))*((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w))/(std::fabs((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w)));
            if (std::isnan(sita) == true)
            {
                sita = 0.0;
            }
            if ((std::fabs(sita)) > M_PI)
            {
                sita = (2*M_PI - std::fabs(sita))*(((robot_pose.orientation.z)*(robot_pose.orientation.w))/(std::fabs((robot_pose.orientation.z)*(robot_pose.orientation.w))));
            }
            // position_x = odom.pose.pose.position.x + missed.position.x;
            // position_y = odom.pose.pose.position.y + missed.position.y;
            // position_z = odom.pose.pose.position.z + missed.position.z;
            // sita = (2*(acos(odom.pose.pose.orientation.w + missed.orientation.w)))*((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w))/(std::fabs((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w)));
            // if ((std::fabs(sita)) > M_PI)
            // {
            //     sita = (2*M_PI - std::fabs(sita))*(((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w))/(std::fabs((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w))));
            // }
            f = true;
        }
    public:
        geometry_msgs::Pose robot_pose;
        geometry_msgs::Pose missed;
        // float position_x = 100.0, position_y = 100.0, position_z = 0.0, sita;
        float sita;
        ROBOT_POSITION()
        {
            ros::NodeHandle node;
            sub_odom = node.subscribe("/odom", 10, &ROBOT_POSITION::callback_odom, this);
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
        ROBOT_POSITION robot_position;
        int i;
        bool start_frag;
        void callback_obstacle(const sensor_msgs::LaserScan &ob)
        {
            // range.clear();
            ob_sita.clear();
            range_point.clear();
            float a;
            range_angle_increment = ob.angle_increment;
            for (i=0; i<ob.ranges.size(); i++)
            {
                if ((ob.range_min <= ob.ranges[i]) && (ob.ranges[i] <= ob.range_max))
                {
                    a = (robot_position.sita + ob.angle_min + range_angle_increment*i);
                    point.x = robot_position.robot_pose.position.x + (ob.ranges[i]*(cos(a)));
                    point.y = robot_position.robot_pose.position.y + (ob.ranges[i]*(sin(a)));
                    point.z = 0.02;
                    ob_sita.push_back(a);
                    range_point.push_back(point);
                }
            }
            start_frag = true;
        }
    public:
        std::vector<float> ob_sita;
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
            robot_position.missed.position.x = 0.0;
            robot_position.missed.position.y = 0.0;
            robot_position.missed.position.z = 0.0;
            robot_position.missed.orientation.w = 0.0;
            robot_position.missed.orientation.x = 0.0;
            robot_position.missed.orientation.y = 0.0;
            robot_position.missed.orientation.z = 0.0;
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
        ros::Subscriber sub_map;
        ros::Subscriber sub_marker_expansion;
        RvizMarkerLibrary marker_lib;
        ROBOT_POSITION robot_position;
        OBSTACLE_DIST obstacle_dist;
        navigation_stack::MapInformation plot_map;
        std::vector<geometry_msgs::Point> expansion_pose;
        bool map_set_frag,expansion_frag;
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
    public:
        PLOT()
        {
            ros::NodeHandle node;
            pub_marker_map = node.advertise<visualization_msgs::Marker>("/active_map", 10);
            pub_marker_robot = node.advertise<visualization_msgs::Marker>("/active_map", 10);
            pub_marker_ob = node.advertise<visualization_msgs::Marker>("/active_map", 10);
            pub_marker_expansion = node.advertise<visualization_msgs::Marker>("/active_map", 10);
            sub_map = node.subscribe("/mapping", 10, &PLOT::callback_map, this);
            sub_marker_expansion = node.subscribe("/expansion_poses", 10, &PLOT::callback_expansion, this);
            get_plot();
            mapping_plot();
        }
        void get_plot()
        {
            map_set_frag = false;
            expansion_frag = false;
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
            geometry_msgs::Vector3 expansion_size;
            geometry_msgs::Pose pose;
            // geometry_msgs::Pose robot_pose_plot;
            scale.x = gridding.size*1.0;
            scale.y = gridding.size*1.0;
            scale.z = 0.00;
            local_ob_size.x = gridding.size/3;
            local_ob_size.y = gridding.size/3;
            local_ob_size.z = 0.0;
            // expansion_size.x = gridding.size;
            // expansion_size.y = gridding.size;
            // expansion_size.z = 0.001;
            expansion_size.x = gridding.size;
            expansion_size.y = gridding.size;
            expansion_size.z = 0.01;
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
            robot_color.a = 1.0;
            robot_color.r = 0.0;
            robot_color.g = 0.0;
            robot_color.b = 1.0;
            struct timeval t;
            long sec_0, sec;
            pt.z = 0;
            rgb.a = 1.0;
            while (ros::ok())
            {
                points.clear();
                colors.clear();
                ob_colors.clear();
                expansion_colors.clear();
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
                // robot_pose_plot.position.x = robot_position.robot_pose.position.x;
                // robot_pose_plot.position.y = robot_position.robot_pose.position.y;
                // robot_pose_plot.position.z = robot_position.robot_pose.position.z;
                // robot_pose_plot.orientation.w = robot_position.robot_pose.orientation.w*(1/sqrt(2));
                // robot_pose_plot.orientation.x = robot_position.robot_pose.orientation.x;
                // robot_pose_plot.orientation.y = robot_position.robot_pose.orientation.y;
                // robot_pose_plot.orientation.z = robot_position.robot_pose.orientation.z*(-1/sqrt(2));
                pub_marker_robot.publish( marker_lib.makeMarker( visualization_msgs::Marker::ARROW, "map", "arrow", robot_position.robot_pose, robot_size, robot_color, ros::Duration(1) ) );
                rgb.r = 1.0;
                rgb.g = 0.0;
                rgb.b = 0.0;
                ob_colors.resize(obstacle_dist.range_point.size(),rgb);
                pub_marker_ob.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::POINTS, "map", "point_list1", pose, obstacle_dist.range_point, local_ob_size, ob_colors, ros::Duration(1) ) );
                if (expansion_frag)
                {
                    rgb.r = 0.0;
                    rgb.g = 1.0;
                    rgb.b = 0.0;
                    expansion_colors.resize(expansion_pose.size(),rgb);
                    pub_marker_expansion.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::CUBE_LIST, "map", "cube_list2", pose, expansion_pose, expansion_size, expansion_colors, ros::Duration(1) ) );
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