#include <stdio.h>
#include <ros/ros.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <ctime>
#include <limits>
#include <math.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <typeinfo>
#include <sys/time.h>
#include <fstream>
#include <matplotlib-cpp/matplotlibcpp.h>
#include <navigation_stack/MapInformation.h>
#include <navigation_stack/PathPoint.h>

#include <Eigen/Dense>
using namespace Eigen;


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


struct NODE
{
    int x;
    int y;
    float goal_cost = std::numeric_limits<float>::max();
    float heuristic_cost = std::numeric_limits<float>::max();
    float sum_cost = std::numeric_limits<float>::max();
    bool open_node = false;
    // bool search_end = false;
    int px;
    int py;
};


class TEST
{
    public:
        GRIDDING gridding;
        ros::Publisher pub_map;
        ros::Publisher pub_robot_position;
        geometry_msgs::Pose goal_pose;
        ros::Subscriber sub_initial;
        navigation_stack::MapInformation map;
        geometry_msgs::Pose robot_position;
        bool global_path_node_searches = false; // 4近傍ならfalse、8近傍ならtrue
        bool unknown_grid_path = true;
        int plot_size;
        int zero_point;
        std::vector<int> vector_1d;
        std::vector<std::vector<int>> map_cost_global;
        std::vector<std::vector<int>> map_cost_local;
        std::vector<NODE> node_vector;
        // std::vector<NODE> node_1d;
        // std::vector<std::vector<NODE>> node_2d;
        TEST()
        {
            ros::NodeHandle node;
            pub_map = node.advertise<navigation_stack::MapInformation>("/mapping", 10);
            pub_robot_position = node.advertise<geometry_msgs::Pose>("/robot_position", 10);
            robot_position.position.x = -2.000;
            robot_position.position.y = -0.499;
            robot_position.position.z = 0.;
            robot_position.orientation.w = 0.9999925745226904;
            robot_position.orientation.x = -7.319129950051188e-06;
            robot_position.orientation.y = 0.0038531009294144433;
            robot_position.orientation.z = 6.677678890616588e-05;
            pub_robot_position.publish(robot_position);
            sub_initial = node.subscribe("/initialpose", 10, &TEST::callback_initial, this);
            plot_size = (int)((sqrt(std::numeric_limits<int>::max()))/4);
            zero_point = (int)(plot_size/2);
            vector_1d.resize(plot_size,-1);
            // node_1d.resize(plot_size);
            get_map();
            path_plan();
        }
        void callback_initial(const geometry_msgs::PoseWithCovarianceStamped &initial)
        {
            robot_position.position.x = initial.pose.pose.position.x;
            robot_position.position.y = initial.pose.pose.position.y;
            robot_position.position.z = initial.pose.pose.position.z;
            robot_position.orientation.w = initial.pose.pose.orientation.w;
            robot_position.orientation.x = initial.pose.pose.orientation.x;
            robot_position.orientation.y = initial.pose.pose.orientation.y;
            robot_position.orientation.z = initial.pose.pose.orientation.z;
            pub_robot_position.publish(robot_position);
        }
        void path_plan()
        {
            // ---~
            goal_pose.position.x = 0.051961421966552734;
            goal_pose.position.y = 1.8232258558273315;
            goal_pose.position.z = 0.0;
            goal_pose.orientation.w = 1.0;
            goal_pose.orientation.x = 0.0;
            goal_pose.orientation.y = 0.0;
            goal_pose.orientation.z = 0.0;
            // ~---
            navigation_stack::PathPoint global_path;
            bool path_flag = global_path_planning(robot_position.position, goal_pose.position, global_path);
            if (path_flag)
            {
                ROS_INFO("global_path\n");
                for (int i=0; i<global_path.poses.size(); i++)
                {
                    printf("%.2f\t%.2f\n",global_path.poses[i].x,global_path.poses[i].y);
                }
            }
            else
            {
                ROS_ERROR("No Path...\n");
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
            node.heuristic_cost = euclidean_distance(gridding.int_to_grid(node.x - zero_point), gridding.int_to_grid(node.y - zero_point), gridding.float_to_grid(goal_position.x), gridding.float_to_grid(goal_position.y));
            node.sum_cost = node.goal_cost + node.heuristic_cost;
            node.open_node = true;
            node.px = node.x;
            node.py = node.y;
            node_vector.push_back(node);
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
                    ROS_ERROR("NOT PATH\n");
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
                                        if (((map_cost_global[node_vector[select_index].x + i][node_vector[select_index].y + j] == 0) || ((map_cost_global[node_vector[select_index].x + i][node_vector[select_index].y + j] == -1) && (unknown_grid_path))) && (map_cost_local[node_vector[select_index].x + i][node_vector[select_index].y + j] != 1))
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
        void get_map()
        {
            std::string map_file_path = ros::package::getPath("navigation_stack") + "/map/mapping_lecture.yaml";
            std::ifstream file(map_file_path);
            if (file.is_open())
            {
                YAML::Node yaml_data;
                yaml_data = YAML::LoadFile(map_file_path);
                // YAMLからデータを抽出する
                try
                {
                    const YAML::Node& yaml_positions_set_cost = yaml_data["map_data"]["cost"];
                    const YAML::Node& yaml_positions_set_clearly = yaml_data["map_data"]["clearly"];
                    for (int i=0; i<yaml_positions_set_cost.size(); i++)
                    {
                        geometry_msgs::Vector3 vec;
                        vec.x = yaml_positions_set_cost[i]["x"].as<float>();
                        vec.y = yaml_positions_set_cost[i]["y"].as<float>();
                        vec.z = 0.0;
                        map.cost.push_back(vec);
                    }
                    for (int i=0; i<yaml_positions_set_clearly.size(); i++)
                    {
                        geometry_msgs::Vector3 vec;
                        vec.x = yaml_positions_set_clearly[i]["x"].as<float>();
                        vec.y = yaml_positions_set_clearly[i]["y"].as<float>();
                        vec.z = 0.0;
                        map.clearly.push_back(vec);
                    }
                }
                catch (const YAML::Exception& e)
                {
                    printf("\x1b[31m\n%s file is no type...\n\x1b[0m",map_file_path.c_str());
                    ros::spinOnce();
                    ros::spin();
                }
            }
            else
            {
                printf("\x1b[31m\nFailed to open file: %s\n\x1b[0m",map_file_path.c_str());
                ros::spinOnce();
                ros::spin();
            }
            map_cost_global.resize(plot_size,vector_1d);
            map_cost_local.resize(plot_size,vector_1d);
            for (int i=0; i<map.clearly.size(); i++)
            {
                map_cost_global[zero_point + gridding.float_to_int(map.clearly[i].x)][zero_point + gridding.float_to_int(map.clearly[i].y)] = 0;
            }
            for (int i=0; i<map.cost.size(); i++)
            {
                map_cost_global[zero_point + gridding.float_to_int(map.cost[i].x)][zero_point + gridding.float_to_int(map.cost[i].y)] = 1;
                map_cost_local[zero_point + gridding.float_to_int(map.cost[i].x)][zero_point + gridding.float_to_int(map.cost[i].y)] = 1;
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
                if (node1.goal_cost < node1.goal_cost)
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
            node.heuristic_cost = euclidean_distance(gridding.int_to_grid(node.x - zero_point), gridding.int_to_grid(node.y - zero_point), gridding.float_to_grid(gp.x), gridding.float_to_grid(gp.y));
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
    ros::init(argc, argv, "cpp_lecture");
    std::vector<float> x_points = {0, 1, 3};
    std::vector<float> y_points = {1, 5, 13};

    // Initialize matrix for least squares
    MatrixXd A(x_points.size(), 2);
    VectorXd b(x_points.size());

    // Fill the matrix A and vector b
    for (int i = 0; i < x_points.size(); i++) {
        A(i, 0) = x_points[i];
        A(i, 1) = 1.0;
        b(i) = y_points[i];
    }

    // Solve the least squares problem
    Vector2d result = A.colPivHouseholderQr().solve(b);

    // Result contains the slope (result[0]) and the intercept (result[1])
    std::cout << "Slope: " << result[0] << ", Intercept: " << result[1] << "\n";
    
    ros::spinOnce();
    ros::spin();
    return 0;
}