#include <stdio.h>
#include <ros/ros.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
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
            return (float)((s/arg_size) + (size/2.0)*(s/std::fabs(s)));
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
        }
        bool global_path_planning(geometry_msgs::Point init_position, geometry_msgs::Point goal_position, navigation_stack::PathPoint &global_path)
        {
            global_path.poses.clear();
            node_vector.clear();
            NODE node;
            node.x = zero_point + gridding.float_to_int(gridding.float_to_grid(init_position.x));
            node.y = zero_point + gridding.float_to_int(gridding.float_to_grid(init_position.y));
            // node.goal_cost = euclidean_distance(x0, y0, x1, y1);
            node.goal_cost = 0.;
            node.heuristic_cost = euclidean_distance(gridding.int_to_grid(node.x - zero_point), gridding.int_to_grid(node.y - zero_point), gridding.float_to_grid(goal_position.x), gridding.float_to_grid(goal_position.y));
            node.sum_cost = node.goal_cost + node.heuristic_cost;
            node.open_node = true;
            // nodd.search_end = false;
            node.px = node.x;
            node.py = node.y;
            node_vector.push_back(node);
            int limit_point[4] = {zero_point + gridding.float_to_int(gridding.float_to_grid(init_position.x)), zero_point + gridding.float_to_int(gridding.float_to_grid(init_position.x)), zero_point + gridding.float_to_int(gridding.float_to_grid(init_position.y)), zero_point + gridding.float_to_int(gridding.float_to_grid(init_position.y))};
            while (ros::ok())
            {
                bool end_flag = false;
                // float min_sum_cost = std::numeric_limits<float>::max();
                int select_index = -1;
                for (int i=0; i<node_vector.size(); i++)
                {
                    if (node_vector[i].open_node)
                    {
                        select_index = select_node(select_index, i, node_vector[select_index], node_vector[i]);
                    }
                }
                if (select_index == -1)
                {
                    return false;
                }
                for (int i=-1; i<=1; i++)
                {
                    if ((node_vector[select_index].x + i) < limit_point[0])
                    {
                        limit_point[0] = node_vector[select_index].x + i;
                    }
                    if (limit_point[1] < (node_vector[select_index].x + i))
                    {
                        limit_point[1] = node_vector[select_index].x + i;
                    }
                    for (int j=-1; j<=1; j++)
                    {
                        if ((node_vector[select_index].y + j) < limit_point[2])
                        {
                            limit_point[2] = node_vector[select_index].y + j;
                        }
                        if (limit_point[3] < (node_vector[select_index].y + j))
                        {
                            limit_point[3] = node_vector[select_index].y + j;
                        }
                        if ((i!=0) || (j!=0))
                        {
                            if ((global_path_node_searches) || ((i==0) || (j==0)))
                            {
                                if (((map_cost_global[node_vector[select_index].x + i][node_vector[select_index].y + j] == 0) || ((map_cost_global[node_vector[select_index].x + i][node_vector[select_index].y + j] == -1) && (unknown_grid_path))) && (map_cost_local[node_vector[select_index].x + i][node_vector[select_index].y + j] != 1))
                                {
                                    end_flag = node_open(node_vector[select_index].x + i, node_vector[select_index].y + j, node_vector[select_index], node, goal_position);
                                    node_vector.push_back(node);
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
                    if (end_flag)
                    {
                        break;
                    }
                }
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
                for (i=0; i<node_vector.size(); i++)
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
            if (i1 == -1)
            {
                return i2;
            }
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
                    return i2
                }
            }
            else
            {
                return i2;
            }
            return i1;
        }
        bool node_open(int x, int y, NODE node_before, NODE &node, geometry_msgs::Point gp)
        {
            node.x = x;
            node.y = y;
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
    ros::spinOnce();
    ros::spin();
}


// float localization_sita(std::vector<float> me, std::vector<float> fixpose, float fixsita, std::vector<std::vector<float>> ob_global, std::vector<float> obran, std::vector<float> obsita)
// {
//     std::vector<std::vector<float>> ob_local;
//     std::vector<float> oblocal_to_obglobal;
//     float sita_miss = (M_PI/180)*(3);
//     float sita_grid = (M_PI/180)*(0.1);
//     float pose_miss = 2;
//     float oblocal_to_obglobal_min;
//     float var_stack = std::numeric_limits<float>::max();
//     float var, ave;
//     float new_fixsita;
//     bool min_ok_frag;
//     for (float i=(me[2]-sita_miss); i<=(me[2]+sita_miss); i+=sita_grid)
//     {
//         ob_local.clear();
//         for (int j=0; j<obran.size(); j++)
//         {
//             ob_local.push_back({me[0] + obran[j]*cos(i), me[1] + obran[j]*sin(i)});
//         }
//         oblocal_to_obglobal.clear();
//         for (int j=0; j<ob_local.size(); j++)
//         {
//             oblocal_to_obglobal_min = std::numeric_limits<float>::max();
//             min_ok_frag = false;
//             for (int k=0; k<ob_global.size(); k++)
//             {
//                 if ((sqrt((pow((ob_local[j][0]-ob_global[k][0]),2))+(pow((ob_local[j][1]-ob_global[k][1]),2)))) <= ((sqrt(2))*pose_miss))
//                 // if ((sqrt((pow((ob_local[j][0]-ob_global[k][0]),2))+(pow((ob_local[j][1]-ob_global[k][1]),2)))) > (2*pose_miss))
//                 // if (1)
//                 {
//                     continue;
//                 }
//                 else if ((sqrt((pow((ob_local[j][0]-ob_global[k][0]),2))+(pow((ob_local[j][1]-ob_global[k][1]),2)))) < oblocal_to_obglobal_min)
//                 {
//                     oblocal_to_obglobal_min = sqrt((pow((ob_local[j][0]-ob_global[k][0]),2))+(pow((ob_local[j][1]-ob_global[k][1]),2)));
//                     min_ok_frag = true;
//                 }
//             }
//             if (min_ok_frag)
//             {
//                 oblocal_to_obglobal.push_back(oblocal_to_obglobal_min);
//             }
//         }
//         var = 0;
//         ave = 0;
//         for (int j=0; j<oblocal_to_obglobal.size(); j++)
//         {
//             ave += ((oblocal_to_obglobal[j])/(oblocal_to_obglobal.size()));
//         }
//         for (int j=0; j<oblocal_to_obglobal.size(); j++)
//         {
//             var += ((pow((oblocal_to_obglobal[j]-ave),2))/**(oblocal_to_obglobal.size())*/);
//         }
//         if (var < var_stack)
//         {
//             var_stack = var;
//             new_fixsita = i;
//         }
//     }
//     fixsita += (new_fixsita - me[2]);
//     return fixsita;
// }

// std::vector<float> localization_pose(std::vector<float> me, std::vector<float> fixpose, float fixsita, std::vector<std::vector<float>> ob_global, std::vector<std::vector<float>> ob_local)
// {
//     std::vector<float> oblocal_to_obglobal;
//     std::vector<float> new_fixpose{0,0};
//     float pose_miss = 8;
//     float grid = 1;
//     float oblocal_to_obglobal_min;
//     int score, is, js;
//     int score_stack = 0;
//     for (float i=((-1)*pose_miss); i<=pose_miss; i+=grid)
//     {
//         for (float j=((-1)*pose_miss); j<=pose_miss; j+=grid)
//         {
//             score = 0;
//             for (int l=0; l<ob_local.size(); l++)
//             {
//                 for (int m=0; m<ob_global.size(); m++)
//                 {
//                     if ((((ob_global[m][0]-(grid/2)) <= (ob_local[l][0]+i)) && ((ob_local[l][0]+i) < (ob_global[m][0]+(grid/2)))) && (((ob_global[m][1]-(grid/2)) <= (ob_local[l][1]+j)) && ((ob_local[l][1]+j) < (ob_global[m][1]+(grid/2)))))
//                     {
//                         score++;
//                         break;
//                     }
//                 }
//             }
//             if (score_stack < score)
//             {
//                 score_stack = score;
//                 new_fixpose[0] = i;
//                 new_fixpose[1] = j;
//             }
//         }
//     }
//     fixpose[0] += new_fixpose[0];
//     fixpose[1] += new_fixpose[1];
//     return fixpose;
// }






// struct timeval t;   //時間取得用の構造体を定義
// long sec_0, sec;    //時間の差を比較する変数
// ros::spinOnce();
// gettimeofday(&t, NULL);    //時間を取得
// sec_0 = t.tv_sec;     //整数部分の秒(s)
// while (ros::ok())
// {
//     gettimeofday(&t, NULL);    //時間を取得
//     sec = t.tv_sec;     //整数部分の秒(s)
//     ros::spinOnce();
//     pub_start.publish(go);
//     // if (ros::ok()!=true)
//     if ((sec-sec_0) > 3)
//     {
//         break;
//     }
// }