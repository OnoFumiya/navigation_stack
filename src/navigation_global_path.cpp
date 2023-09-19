#include <stdio.h>
#include <ros/ros.h>
#include <unistd.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <string>
#include <iostream>
#include <limits>
#include <math.h>
#include <typeinfo>
#include <sys/time.h>
#include <matplotlib-cpp/matplotlibcpp.h>
#include <navigation_stack/MapInformation.h>
#include <navigation_stack/ExpansionPoints.h>
#include <navigation_stack/PathPoint.h>



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
        float lidar_pose[2] = {0.2, 0.0};
        // float lidar_pose[2] = {0.0, 0.0};
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



struct NODE
{
    float x;
    float y;
    float goal_cost = std::numeric_limits<float>::max();
    // float heuristic_cost = std::numeric_limits<float>::max();
    float heuristic_cost = 0.;
    float sum_cost = std::numeric_limits<float>::max();
    bool open_node = false;
    int px;
    int py;
};


class PATH_PLANNING
{
    private:
        GRIDDING gridding;
        OBSTACLE_DIST obstacle_dist;
        ROBOT_POSITION robot_position;
        float global_cost_range = 0.20;
        std::string global_path_mode = "A_STAR";
        // std::string global_path_mode = "Dijkstra";
        bool global_path_node_searches = true; // 4近傍ならfalse、8近傍ならtrue
        bool unknown_grid_path = true;
        std_msgs::Bool goal_flag;
        ros::Publisher pub_global_path;
        ros::Subscriber sub_map;
        ros::Subscriber sub_goal_flag;
        ros::Subscriber sub_goal_2dnav;
        navigation_stack::MapInformation map;
        bool map_set_frag;
        geometry_msgs::Pose goal_pose;
        int plot_size;
        int zero_point;
        std::vector<int> vector_1d;
        std::vector<std::vector<int>> map_cost_global;
        std::vector<std::vector<int>> map_cost_local;
        std::vector<NODE> node_vector;
        int id;
        void callback_map(const navigation_stack::MapInformation &get_map)
        {
            map.cost.clear();
            map.clearly.clear();
            for (int i=0; i<get_map.cost.size(); i++)
            {
                map.cost.push_back(get_map.cost[i]);
            }
            for (int i=0; i<get_map.clearly.size(); i++)
            {
                map.clearly.push_back(get_map.clearly[i]);
            }
            map_set_frag = true;
        }
        void callback_goalflag(const std_msgs::Bool &get_flag)
        {
            goal_flag.data = get_flag.data;
        }
        void callback_2dnav(const geometry_msgs::PoseStamped nav)
        {
            id = 0;
            goal_pose.position.x = nav.pose.position.x;
            goal_pose.position.y = nav.pose.position.y;
            goal_pose.position.z = nav.pose.position.z;
            goal_pose.orientation.w = nav.pose.orientation.w;
            goal_pose.orientation.x = nav.pose.orientation.x;
            goal_pose.orientation.y = nav.pose.orientation.y;
            goal_pose.orientation.z = nav.pose.orientation.z;
            goal_flag.data = false;
        }
    public:
        PATH_PLANNING()
        {
            ros::NodeHandle node;
            pub_global_path = node.advertise<navigation_stack::PathPoint>("/global_path_planning", 10);
            sub_map = node.subscribe("/mapping", 10, &PATH_PLANNING::callback_map, this);
            sub_goal_flag = node.subscribe("/goal_flag", 10, &PATH_PLANNING::callback_goalflag, this);
            sub_goal_2dnav = node.subscribe("/move_base_simple/goal", 10, &PATH_PLANNING::callback_2dnav, this);
            plot_size = (int)((sqrt(std::numeric_limits<int>::max()))/4);
            zero_point = (int)(plot_size/2);
            goal_flag.data = true;
            wait_map();
            path_plan_control();
        }
        void wait_map()
        {
            vector_1d.resize(plot_size,-1);
            map_set_frag = false;
            ros::spinOnce();
            while (ros::ok())
            {
                ros::spinOnce();
                if (map_set_frag)
                {
                    ros::spinOnce();
                    break;
                }
            }
            ros::spinOnce();
        }
        void path_plan_control()
        {
            while (ros::ok())
            {
                ros::spinOnce();
                if (goal_flag.data)
                {
                    ros::spinOnce();
                    continue;
                }
                else
                {
                    ros::spinOnce();
                    path_plan();
                }
                ros::spinOnce();
            }
        }
        void path_plan()
        {
            set_vector_globalmap();
            navigation_stack::PathPoint global_path;
            while (ros::ok())
            {
                ros::spinOnce();
                // set_vector_globalmap();
                set_vector_localmap();
                set_vector_initialpoint();
                if (goal_flag.data)
                {
                    global_path.goal_pose.position.x = robot_position.robot_pose.position.x;
                    global_path.goal_pose.position.y = robot_position.robot_pose.position.y;
                    global_path.goal_pose.position.z = robot_position.robot_pose.position.z;
                    global_path.goal_pose.orientation.w = robot_position.robot_pose.orientation.w;
                    global_path.goal_pose.orientation.x = robot_position.robot_pose.orientation.x;
                    global_path.goal_pose.orientation.y = robot_position.robot_pose.orientation.y;
                    global_path.goal_pose.orientation.z = robot_position.robot_pose.orientation.z;
                    global_path.poses.clear();
                    pub_global_path.publish(global_path);
                    return;
                }
                global_path.goal_pose.position.x = goal_pose.position.x;
                global_path.goal_pose.position.y = goal_pose.position.y;
                global_path.goal_pose.position.z = goal_pose.position.z;
                global_path.goal_pose.orientation.w = goal_pose.orientation.w;
                global_path.goal_pose.orientation.x = goal_pose.orientation.x;
                global_path.goal_pose.orientation.y = goal_pose.orientation.y;
                global_path.goal_pose.orientation.z = goal_pose.orientation.z;
                bool path_flag = global_path_planning(robot_position.robot_pose.position, goal_pose.position, global_path);
                ros::spinOnce();
                if (path_flag)
                {
                    ROS_INFO("global_path is publish\n");
                    global_path.id = id;
                    id++;
                }
                else
                {
                    global_path.poses.clear();
                    ROS_ERROR("No Path...\n");
                }
                pub_global_path.publish(global_path);
                ros::spinOnce();
                // ros::spin();
                // ros::Duration(2).sleep();
            }
        }
        bool global_path_planning(geometry_msgs::Point init_position, geometry_msgs::Point goal_position, navigation_stack::PathPoint &global_path)
        {
            global_path.poses.clear();
            node_vector.clear();
            NODE node;
            node.x = zero_point + gridding.float_to_int(gridding.float_to_grid(init_position.x));
            node.y = zero_point + gridding.float_to_int(gridding.float_to_grid(init_position.y));
            node.goal_cost = 0.;
            // node.heuristic_cost = euclidean_distance(gridding.int_to_grid(node.x - zero_point), gridding.int_to_grid(node.y - zero_point), gridding.float_to_grid(goal_position.x), gridding.float_to_grid(goal_position.y));
            node.heuristic_cost = 0.;
            node.sum_cost = node.goal_cost + node.heuristic_cost;
            node.open_node = true;
            node.px = node.x;
            node.py = node.y;
            ros::spinOnce();
            node_vector.push_back(node);
            ros::spinOnce();
            while (ros::ok())
            {
                bool end_flag = false;
                int select_index = -1;
                for (int i=0; i<node_vector.size(); i++)
                {
                    if (node_vector[i].open_node)
                    {
                        if (select_index == -1)
                        {
                            select_index = i;
                        }
                        else
                        {
                            select_index = select_node(select_index, i, node_vector[select_index], node_vector[i]);
                        }
                    }
                }
                if (select_index == -1)
                {
                    return false;
                }
                for (int i=-1; i<=1; i++)
                {
                    if ((0 <= (node_vector[select_index].x + i)) && ((node_vector[select_index].x + i) < plot_size))
                    {
                        for (int j=-1; j<=1; j++)
                        {
                            if ((0 <= (node_vector[select_index].y + j)) && ((node_vector[select_index].y + j) < plot_size))
                            {
                                if ((i!=0) || (j!=0))
                                {
                                    if ((global_path_node_searches) || ((i==0) || (j==0)))
                                    {
                                        if ((map_cost_local[node_vector[select_index].x + i][node_vector[select_index].y + j] == 0) || (((map_cost_global[node_vector[select_index].x + i][node_vector[select_index].y + j] == 0) || ((map_cost_global[node_vector[select_index].x + i][node_vector[select_index].y + j] == -1) && (unknown_grid_path))) && (map_cost_local[node_vector[select_index].x + i][node_vector[select_index].y + j] != 1)))
                                        {
                                            node.x = node_vector[select_index].x + i;
                                            node.y = node_vector[select_index].y + j;
                                            bool open_is = node_open_checker(select_index, node, node_vector);
                                            if (open_is)
                                            {
                                                end_flag = node_open(node_vector[select_index], node, goal_position);
                                                node_vector.push_back(node);
                                            }
                                            if (end_flag)
                                            {
                                                break;
                                            }
                                        }
                                    }
                                }
                                if (end_flag)
                                {
                                    break;
                                }
                            }
                        }
                    }
                    if (end_flag)
                    {
                        break;
                    }
                }
                node_vector[select_index].open_node = false;
                if (end_flag)
                {
                    break;
                }
            }
            geometry_msgs::Vector3 vec;
            int parent_node_x;
            int parent_node_y;
            vec.x = gridding.int_to_grid(node_vector[node_vector.size() - 1].x - zero_point);
            vec.y = gridding.int_to_grid(node_vector[node_vector.size() - 1].y - zero_point);
            vec.z = 0.;
            parent_node_x = node_vector[node_vector.size() - 1].px;
            parent_node_y = node_vector[node_vector.size() - 1].py;
            global_path.poses.insert(global_path.poses.begin(), vec);
            while (ros::ok())
            {
                int select_index;
                for (int i=0; i<node_vector.size(); i++)
                {
                    if ((parent_node_x == node_vector[i].x) && (parent_node_y == node_vector[i].y))
                    {
                        vec.x = gridding.int_to_grid(node_vector[i].x - zero_point);
                        vec.y = gridding.int_to_grid(node_vector[i].y - zero_point);
                        global_path.poses.insert(global_path.poses.begin(), vec);
                        select_index = i;
                        break;
                    }
                }
                if (select_index == 0)
                {
                    break;
                }
                else 
                {
                    parent_node_x = node_vector[select_index].px;
                    parent_node_y = node_vector[select_index].py;
                }
            }
            return true;
        }
        void set_vector_globalmap()
        {
            map_cost_global.clear();
            map_cost_global.resize(plot_size,vector_1d);
            for (int i=0; i<map.clearly.size(); i++)
            {
                map_cost_global[zero_point + gridding.float_to_int(map.clearly[i].x)][zero_point + gridding.float_to_int(map.clearly[i].y)] = 0;
            }
            for (int i=0; i<map.cost.size(); i++)
            {
                map_cost_global[zero_point + gridding.float_to_int(map.cost[i].x)][zero_point + gridding.float_to_int(map.cost[i].y)] = 1;
                for (int j=(-1)*((int)(global_cost_range/gridding.size + 0.5)); j<=(int)(global_cost_range/gridding.size + 0.5); j++)
                {
                    if ((0 <= (zero_point + gridding.float_to_int(map.cost[i].x) + j)) && ((zero_point + gridding.float_to_int(map.cost[i].x) + j) < plot_size))
                    {
                        for (int k=(-1)*((int)(global_cost_range/gridding.size + 0.5)); k<=(int)(global_cost_range/gridding.size + 0.5); k++)
                        {
                            if ((0 <= (zero_point + gridding.float_to_int(map.cost[i].y) + k)) && ((zero_point + gridding.float_to_int(map.cost[i].y) + k) < plot_size))
                            {
                                if (euclidean_distance(map.cost[i].x, map.cost[i].y, (map.cost[i].x + gridding.int_to_grid(j)), (map.cost[i].y + gridding.int_to_grid(k))) <= global_cost_range)
                                {
                                    map_cost_global[zero_point + gridding.float_to_int(map.cost[i].x) + j][zero_point + gridding.float_to_int(map.cost[i].y) + k] = 1;
                                }
                            }
                        }
                    }
                }
            }
        }
        void set_vector_localmap()
        {
            map_cost_local.clear();
            map_cost_local.resize(plot_size,vector_1d);
            for (int i=0; i<obstacle_dist.range_point.size(); i++)
            {
                map_cost_local[zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].x))][zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].y))] = 1;
                for (int j=(-1)*((int)(global_cost_range/gridding.size + 0.5)); j<=(int)(global_cost_range/gridding.size + 0.5); j++)
                {
                    if ((0 <= (zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].x)) + j)) && ((zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].x)) + j) < plot_size))
                    {
                        for (int k=(-1)*((int)(global_cost_range/gridding.size + 0.5)); k<=(int)(global_cost_range/gridding.size + 0.5); k++)
                        {
                            if ((0 <= (zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].y)) + k)) && ((zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].y)) + k) < plot_size))
                            {
                                if (euclidean_distance(gridding.float_to_grid(obstacle_dist.range_point[i].x), gridding.float_to_grid(obstacle_dist.range_point[i].y), (gridding.float_to_grid(obstacle_dist.range_point[i].x) + gridding.int_to_grid(j)), (gridding.float_to_grid(obstacle_dist.range_point[i].y) + gridding.int_to_grid(k))) <= global_cost_range)
                                {
                                    map_cost_local[zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].x)) + j][zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i].y)) + k] = 1;
                                }
                            }
                        }
                    }
                }
            }
            for (int i=0; i<obstacle_dist.range_point.size(); i++)
            {
                if ((((M_PI/4)) < std::fabs(obstacle_dist.ob_theta[i])) && (std::fabs(obstacle_dist.ob_theta[i]) < ((3*M_PI/4))))
                {
                    for (int j=1; j<((std::fabs(obstacle_dist.range_point[i].y - robot_position.robot_pose.position.y))/gridding.size); j++)
                    {
                        if (map_cost_local[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.x + ((j*gridding.size/tan(obstacle_dist.ob_theta[i]))*((std::fabs(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y))/(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.y + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y))/(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y)))))] != 1)
                        {
                            map_cost_local[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.x + ((j*gridding.size/tan(obstacle_dist.ob_theta[i]))*((std::fabs(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y))/(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.y + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y))/(obstacle_dist.range_point[i].y-robot_position.robot_pose.position.y)))))] = 0;
                        }
                    }
                }
                else
                {
                    for (int j=1; j<((std::fabs(obstacle_dist.range_point[i].x - robot_position.robot_pose.position.x))/gridding.size); j++)
                    {
                        if (map_cost_local[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.x + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x))/(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.y + ((j*gridding.size*tan(obstacle_dist.ob_theta[i]))*((std::fabs(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x))/(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x)))))] != 1)
                        {
                            map_cost_local[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.x + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x))/(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.y + ((j*gridding.size*tan(obstacle_dist.ob_theta[i]))*((std::fabs(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x))/(obstacle_dist.range_point[i].x-robot_position.robot_pose.position.x)))))] = 0;
                        }
                    }
                }
            }
        }
        void set_vector_initialpoint()
        {
            for (int i=(-1)*((int)(global_cost_range/gridding.size + 0.5)); i<=(int)(global_cost_range/gridding.size + 0.5); i++)
            {
                if ((0 <= (zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.x)) + i)) && ((zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.x)) + i) < plot_size))
                {
                    for (int j=(-1)*((int)(global_cost_range/gridding.size + 0.5)); j<=(int)(global_cost_range/gridding.size + 0.5); j++)
                    {
                        if ((0 <= (zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.y)) + j)) && ((zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.y)) + j) < plot_size))
                        {
                            map_cost_local[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.x)) + i][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.y)) + j] = 0;
                            // map_cost_global[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.x)) + i][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.robot_pose.position.y)) + j] = 0;
                        }
                    }
                }
            }
        }
        float euclidean_distance(float x0, float y0, float x1, float y1)
        {
            return (sqrtf(powf((x1 - x0), 2.) + powf((y1 - y0), 2.)));
        }
        int select_node(int i1, int i2, NODE node1, NODE node2)
        {
            if (node1.sum_cost < node2.sum_cost)
            {
                return i1;
            }
            else if (node1.sum_cost == node2.sum_cost)
            {
                // if (node1.goal_cost > node2.goal_cost)
                if (node1.goal_cost < node2.goal_cost)
                {
                    return i1;
                }
                else
                {
                    return i2;
                }
            }
            else
            {
                return i2;
            }
            return i1;
        }
        bool node_open_checker(int index, NODE check_node, std::vector<NODE> stack_node_vector)
        {
            for (int i=0; i<stack_node_vector.size(); i++)
            {
                if ((index != i) && (stack_node_vector[i].x == check_node.x) && (stack_node_vector[i].y == check_node.y))
                {
                    return false;
                }
            }
            return true;
        }
        bool node_open(NODE node_before, NODE &node, geometry_msgs::Point gp)
        {
            node.goal_cost = node_before.goal_cost + euclidean_distance(gridding.int_to_grid(node_before.x - zero_point), gridding.int_to_grid(node_before.y - zero_point), gridding.int_to_grid(node.x - zero_point), gridding.int_to_grid(node.y - zero_point));
            // node.heuristic_cost = euclidean_distance(gridding.int_to_grid(node.x - zero_point), gridding.int_to_grid(node.y - zero_point), gridding.float_to_grid(gp.x), gridding.float_to_grid(gp.y));
            node.heuristic_cost = 0.;
            node.sum_cost = node.goal_cost + node.heuristic_cost;
            node.open_node = true;
            node.px = node_before.x;
            node.py = node_before.y;
            if (euclidean_distance(gridding.int_to_grid(node.x - zero_point), gridding.int_to_grid(node.y - zero_point), gridding.float_to_grid(gp.x), gridding.float_to_grid(gp.y)) < gridding.size)
            {
                return true;
            }
            return false;
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_global_path");
    PATH_PLANNING path_planning;
    ros::spin();
}