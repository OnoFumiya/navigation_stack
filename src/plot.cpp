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
        // float lidar_pose[2] = {0.2, 0.0};
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
        std::vector<geometry_msgs::Point> range_point;
        float range_angle_increment;
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
        ros::Publisher pub_marker_localpath;
        ros::Subscriber sub_map;
        ros::Subscriber sub_marker_expansion;
        ros::Subscriber sub_marker_globalpath;
        ros::Subscriber sub_marker_localpath;
        RvizMarkerLibrary marker_lib;
        ROBOT_POSITION robot_position;
        OBSTACLE_DIST obstacle_dist;
        navigation_stack::MapInformation plot_map;
        std::vector<geometry_msgs::Point> expansion_pose;
        std::vector<geometry_msgs::Point> globalpath_pose;
        std::vector<geometry_msgs::Point> localpath_pose;
        bool map_set_frag, expansion_frag, globalpath_frag, localpath_frag;
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
            pt_e.z = 0.01;
            globalpath_pose.clear();
            for (int i=0; i<gpp.poses.size(); i++)
            {
                pt_e.x = gpp.poses[i].x;
                pt_e.y = gpp.poses[i].y;
                globalpath_pose.push_back(pt_e);
            }
            globalpath_frag = true;
        }
        void callback_localpath(const navigation_stack::PathPoint &lpp)
        {
            geometry_msgs::Point pt_e;
            pt_e.z = 0.05;
            localpath_pose.clear();
            for (int i=0; i<lpp.poses.size(); i++)
            {
                pt_e.x = lpp.poses[i].x;
                pt_e.y = lpp.poses[i].y;
                localpath_pose.push_back(pt_e);
            }
            localpath_frag = true;
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
            pub_marker_localpath = node.advertise<visualization_msgs::Marker>("/active_map", 10);
            sub_map = node.subscribe("/mapping", 10, &PLOT::callback_map, this);
            sub_marker_expansion = node.subscribe("/expansion_poses", 10, &PLOT::callback_expansion, this);
            sub_marker_globalpath = node.subscribe("/global_path_planning", 10, &PLOT::callback_globalpath, this);
            sub_marker_localpath = node.subscribe("/local_path_planning", 10, &PLOT::callback_localpath, this);
            get_plot();
            mapping_plot();
        }
        void get_plot()
        {
            map_set_frag = false;
            expansion_frag = false;
            globalpath_frag = false;
            localpath_frag = false;
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
            std::vector<geometry_msgs::Point> map_points;
            std::vector<std_msgs::ColorRGBA> map_colors;
            geometry_msgs::Vector3 scale;
            geometry_msgs::Vector3 local_ob_size;
            geometry_msgs::Vector3 robot_size;
            geometry_msgs::Vector3 grid_size;
            geometry_msgs::Vector3 line_size;
            geometry_msgs::Pose base_pose;
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
            line_size.x = gridding.size;
            line_size.y = gridding.size;
            line_size.z = 0.02;
            robot_size.x = 0.3;
            robot_size.y = 0.1;
            robot_size.z = 0.1;
            base_pose.position.x = 0.0;
            base_pose.position.y = 0.0;
            base_pose.position.z = 0.0;
            base_pose.orientation.w = 1.0;
            base_pose.orientation.x = 0.0;
            base_pose.orientation.y = 0.0;
            base_pose.orientation.z = 0.0;
            geometry_msgs::Point pt;
            std_msgs::ColorRGBA black, white, red, blue, cyan, yellow, green, purple, orange;
            black.a = 1.;
            black.r = 0.;
            black.g = 0.;
            black.b = 0.;
            white.a = 1.;
            white.r = 1.;
            white.g = 1.;
            white.b = 1.;
            red.a = 1.;
            red.r = 1.;
            red.g = 0.;
            red.b = 0.;
            blue.a = 1.;
            blue.r = 0.;
            blue.g = 0.;
            blue.b = 1.;
            cyan.a = 1.;
            cyan.r = 0.;
            cyan.g = 1.;
            cyan.b = 1.;
            yellow.a = 1.;
            yellow.r = 1.;
            yellow.g = 1.;
            yellow.b = 0.;
            green.a = 1.;
            green.r = 0.;
            green.g = 1.;
            green.b = 0.;
            purple.a = 1.;
            purple.r = 0.5;
            purple.g = 0.;
            purple.b = 0.5;
            orange.a = 1.;
            orange.r = 1.;
            orange.g = 0.66;
            orange.b = 0.;
            std::vector<std_msgs::ColorRGBA> ob_colors;
            std::vector<std_msgs::ColorRGBA> expansion_colors;
            std::vector<std_msgs::ColorRGBA> globalpath_colors;
            std::vector<std_msgs::ColorRGBA> localpath_colors;
            pt.z = 0;
            while (ros::ok())
            {
                map_points.clear();
                map_colors.clear();
                ob_colors.clear();
                expansion_colors.clear();
                globalpath_colors.clear();
                localpath_colors.clear();
                for (int i=0; i<plot_map.cost.size(); i++)
                {
                    pt.x = plot_map.cost[i].x;
                    pt.y = plot_map.cost[i].y;
                    map_points.push_back(pt);
                    map_colors.push_back(black);
                }
                for (int i=0; i<plot_map.clearly.size(); i++)
                {
                    pt.x = plot_map.clearly[i].x;
                    pt.y = plot_map.clearly[i].y;
                    map_points.push_back(pt);
                    map_colors.push_back(white);
                }
                ros::spinOnce();
                pub_marker_map.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::CUBE_LIST, "map", "map", base_pose, map_points, scale, map_colors, ros::Duration(1) ) );
                pub_marker_robot.publish( marker_lib.makeMarker( visualization_msgs::Marker::ARROW, "map", "robot_position", robot_position.robot_pose, robot_size, yellow, ros::Duration(1) ) );
                ob_colors.resize(obstacle_dist.range_point.size(), red);
                pub_marker_ob.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::POINTS, "map", "laser_point", base_pose, obstacle_dist.range_point, local_ob_size, ob_colors, ros::Duration(1) ) );
                if ((expansion_frag) && (expansion_pose.size()!=0))
                {
                    expansion_colors.resize(expansion_pose.size(), cyan);
                    pub_marker_expansion.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::CUBE_LIST, "map", "expansion_point", base_pose, expansion_pose, grid_size, expansion_colors, ros::Duration(1) ) );
                }
                if ((globalpath_frag) && (globalpath_pose.size()!=0))
                {
                    globalpath_colors.resize(globalpath_pose.size(), green);
                    pub_marker_globalpath.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::CUBE_LIST, "map", "global_path", base_pose, globalpath_pose, grid_size, globalpath_colors, ros::Duration(1) ) );
                }
                if ((localpath_frag) && (localpath_pose.size()!=0))
                {
                    localpath_colors.resize(localpath_pose.size(), blue);
                    pub_marker_localpath.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::LINE_STRIP, "map", "local_path", base_pose, localpath_pose, line_size, localpath_colors, ros::Duration(1) ) );
                }
                ros::spinOnce();
            }
        }
        // void set_vector_globalmap()
        // {
        //     map_cost_global.clear();
        //     for (int i=0; i<map.clearly.size(); i++)
        //     {
        //         map_cost_global[zero_point + gridding.float_to_int(map.clearly[i].x)][zero_point + gridding.float_to_int(map.clearly[i].y)] = 0;
        //     }
        //     for (int i=0; i<map.cost.size(); i++)
        //     {
        //         map_cost_global[zero_point + gridding.float_to_int(map.cost[i].x)][zero_point + gridding.float_to_int(map.cost[i].y)] = 1;
        //         for (int j=(-1)*((int)(global_cost_range/gridding.size + 0.5)); j<=(int)(global_cost_range/gridding.size + 0.5); j++)
        //         {
        //             if ((0 <= (zero_point + gridding.float_to_int(map.cost[i].x) + j)) && ((zero_point + gridding.float_to_int(map.cost[i].x) + j) < plot_size))
        //             {
        //                 for (int k=(-1)*((int)(global_cost_range/gridding.size + 0.5)); k<=(int)(global_cost_range/gridding.size + 0.5); k++)
        //                 {
        //                     if ((0 <= (zero_point + gridding.float_to_int(map.cost[i].y) + k)) && ((zero_point + gridding.float_to_int(map.cost[i].y) + k) < plot_size))
        //                     {
        //                         if (euclidean_distance(map.cost[i].x, map.cost[i].y, (map.cost[i].x + gridding.int_to_grid(j)), (map.cost[i].y + gridding.int_to_grid(k))) <= global_cost_range)
        //                         {
        //                             map_cost_global[zero_point + gridding.float_to_int(map.cost[i].x) + j][zero_point + gridding.float_to_int(map.cost[i].y) + k] = 1;
        //                         }
        //                     }
        //                 }
        //             }
        //         }
        //     }
        // }
        // void set_vector_localmap()
        // {
        //     map_cost_local.clear();
        //     map_cost_local.resize(plot_size,vector_1d);
        //     for (int i=0; i<obstacle_dist.range_point.size(); i++)
        //     {
        //         map_cost_local[zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].x))][zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].y))] = 1;
        //         for (int j=(-1)*((int)(global_cost_range/gridding.size + 0.5)); j<=(int)(global_cost_range/gridding.size + 0.5); j++)
        //         {
        //             if ((0 <= (zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].x)) + j)) && ((zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].x)) + j) < plot_size))
        //             {
        //                 for (int k=(-1)*((int)(global_cost_range/gridding.size + 0.5)); k<=(int)(global_cost_range/gridding.size + 0.5); k++)
        //                 {
        //                     if ((0 <= (zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].y)) + k)) && ((zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].y)) + k) < plot_size))
        //                     {
        //                         if (euclidean_distance(gridding.float_to_grid(obstacle_dist.range_point[i].x), gridding.float_to_grid(obstacle_dist.range_point[i].y), (gridding.float_to_grid(obstacle_dist.range_point[i].x) + gridding.int_to_grid(j)), (gridding.float_to_grid(obstacle_dist.range_point[i].y) + gridding.int_to_grid(k))) <= global_cost_range)
        //                         {
        //                             map_cost_local[zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].x)) + j][zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].y)) + k] = 1;
        //                         }
        //                     }
        //                 }
        //             }
        //         }
        //     }
        //     for (int i=0; i<obstacle_dist.range_point.size(); i++)
        //     {
        //         if ((((M_PI/4)) < std::fabs(obstacle_dist.ob_theta[i])) && (std::fabs(obstacle_dist.ob_theta[i]) < ((3*M_PI/4))))
        //         {
        //             for (int j=1; j<((std::fabs(obstacle_dist.range_point[i].y - robot_position.robot_pose.position.y))/gridding.size); j++)
        //             {
        //                 if (map_cost_local[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.x + ((j*gridding.size/tan(obstacle_dist.ob_theta[i]))*((std::fabs(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y))/(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.y + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y))/(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y)))))] != 1)
        //                 {
        //                     map_cost_local[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.x + ((j*gridding.size/tan(obstacle_dist.ob_theta[i]))*((std::fabs(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y))/(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.y + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y))/(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y)))))] = 0;
        //                 }
        //             }
        //         }
        //         else
        //         {
        //             for (int j=1; j<((std::fabs(obstacle_dist.range_point[i].x - robot_position.robot_pose.position.x))/gridding.size); j++)
        //             {
        //                 if (map_cost_local[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.x + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x))/(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.y + ((j*gridding.size*tan(obstacle_dist.ob_theta[i]))*((std::fabs(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x))/(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x)))))] != 1)
        //                 {
        //                     map_cost_local[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.x + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x))/(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.y + ((j*gridding.size*tan(obstacle_dist.ob_theta[i]))*((std::fabs(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x))/(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x)))))] = 0;
        //                 }
        //             }
        //         }
        //     }
        // }
};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_plot");
    PLOT plot;
    ros::spin();
    return 0;
}