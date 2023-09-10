#include <stdio.h>
#include <ros/ros.h>
#include <unistd.h>
#include <std_msgs/Bool.h>
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
#include <math.h>
#include <typeinfo>
#include <sys/time.h>
#include <matplotlib-cpp/matplotlibcpp.h>
#include <navigation_stack/MapInformation.h>
#include <navigation_stack/ExpansionPoints.h>



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
        bool f;
        void callback_robot(const geometry_msgs::Pose &robot)
        {
            robot_pose.position.x = robot.position.x;
            robot_pose.position.y = robot.position.y;
            robot_pose.position.z = robot.position.z + 0.03;
            robot_theta = (2*(acos(robot.orientation.w)))*((robot.orientation.z)*(robot.orientation.w))/(std::fabs((robot.orientation.z)*(robot.orientation.w)));
            if (std::isnan(robot_theta) == true)
            {
                robot_theta = 0.0;
            }
            if ((std::fabs(robot_theta)) > M_PI)
            {
                robot_theta = (2*M_PI - std::fabs(robot_theta))*(((robot.orientation.z)*(robot.orientation.w))/(std::fabs((robot.orientation.z)*(robot.orientation.w))));
            }
            f = true;
        }
    public:
        geometry_msgs::Pose robot_pose;
        float robot_theta;
        ROBOT_POSITION()
        {
            ros::NodeHandle node;
            sub_odom = node.subscribe("/robot_position", 10, &ROBOT_POSITION::callback_robot, this);
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


class MOVE_CLASS
{
    private:
        ros::Publisher pub_twist;
    public:
        MOVE_CLASS()
        {
            ros::NodeHandle node;
            pub_twist = node.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 10);
        }
        void straight_and_turn_time(float u_vel, float u_ang, float dt)
        {
            ros::Rate rate(5.0);
            geometry_msgs::Twist vel;
            struct timeval t;
            int i = 0;
            while (1)
            {
                if (dt-i >= 1)
                {
                    i++;
                }
                else
                {
                    break;
                }
            }
            long udt = (dt-i)*(pow(10,6));
            dt = i;
            vel.linear.x = u_vel;
            vel.angular.z = u_ang*0.99;
            gettimeofday(&t, NULL);    //時間を取得
            long sec_0 = t.tv_sec , sec;     //整数部分の秒(s)
            long usec_0 = t.tv_usec, usec;   //小数以下第6位までの((10**(-6))s)
            ros::spinOnce();
            while (ros::ok())
            {
                ros::spinOnce();
                pub_twist.publish(vel);
                gettimeofday(&t, NULL);
                sec = t.tv_sec;     //整数部分の秒(s)
                usec = t.tv_usec;   //小数以下第6位までの((10**(-6))s)
                if (((sec - sec_0) >= dt) || (dt == 0))
                {
                    if ((udt == 0) || (((usec - usec_0) >= 0) && (usec - usec_0) >= udt) || (((usec - usec_0) < 0) && ((usec+(pow(10,6))-usec_0) >= udt)))
                    {
                        break;
                    }
                }
            }
        }
        void stop_vel()
        {
            geometry_msgs::Twist vel;
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
            struct timeval t;
            gettimeofday(&t, NULL);    //時間を取得
            long sec_0 = t.tv_sec, sec;     //整数部分の秒(s)
            while (ros::ok())
            {
                gettimeofday(&t, NULL);    //時間を取得
                sec = t.tv_sec;     //整数部分の秒(s)
                ros::spinOnce();
                pub_twist.publish(vel);
                if ((sec-sec_0) > 3)
                {
                    break;
                }
            }
        }
        void go(float st, float si, float t)
        {
            ros::spinOnce();
            straight_and_turn_time(st, si, t);
            stop_vel();
            ros::spinOnce();
        }
};


class EXPANSION_POINTER
{
    private:
        ros::Publisher pub_expansion;
        ros::Subscriber sub_map;
        GRIDDING gridding;
        int plot_size;
        int zero_point;
        int limit_point_dijkstra[4];
        float max_float;
        std::vector<int> vector_1d_int;
        std::vector<float> vector_1d_float;
        std::vector<bool> vector_1d_bool;
        std::vector<std::vector<int>> vector_2d;
        std::vector<std::vector<float>> vector_2d_dijkstra_cost;
        std::vector<std::vector<bool>> vector_2d_dijkstra_bool;
        navigation_stack::MapInformation get_map_stack;
        navigation_stack::ExpansionPoints expansion_pose;
        bool f;
        void callback_map(const navigation_stack::MapInformation &get_map)
        {
            get_map_stack.cost.clear();
            get_map_stack.clearly.clear();
            for (int i=0; i<get_map.cost.size(); i++)
            {
                get_map_stack.cost.push_back(get_map.cost[i]);
            }
            for (int i=0; i<get_map.clearly.size(); i++)
            {
                get_map_stack.clearly.push_back(get_map.clearly[i]);
            }
            f = true;
        }
    public:
        EXPANSION_POINTER()
        {
            ros::NodeHandle node;
            plot_size = (int)((sqrt(std::numeric_limits<int>::max()))/4);
            max_float = (float)((sqrt(std::numeric_limits<float>::max()))/3);
            zero_point = (int)(plot_size/2);
            vector_1d_float.resize(plot_size, max_float);
            vector_1d_int.resize(plot_size,-1);
            vector_1d_bool.resize(plot_size,false);
            sub_map = node.subscribe("/mapping", 10, &EXPANSION_POINTER::callback_map, this);
            pub_expansion = node.advertise<navigation_stack::ExpansionPoints>("/expansion_poses",10);
            get_sub_map();
            dijkstra();
        }
        void get_sub_map()
        {
            ros::spinOnce();
            f = false;
            while (ros::ok())
            {
                ros::spinOnce();
                if (f)
                {
                    ros::spinOnce();
                    break;
                }
                ros::spinOnce();
            }
            ros::spinOnce();
        }
        void dijkstra()
        {
            ROBOT_POSITION robot_position;
            geometry_msgs::Pose pose;
            float min_cost;
            int min_x, min_y;
            int dijkstra_start_x, dijkstra_start_y;
            bool push_frag;

            while (ros::ok())
            {
                ros::spinOnce();
                expansion_pose.poses.clear();
                vector_2d.clear();
                vector_2d.resize(plot_size,vector_1d_int);
                vector_2d_dijkstra_cost.clear();
                vector_2d_dijkstra_cost.resize(plot_size,vector_1d_float);
                vector_2d_dijkstra_bool.clear();
                vector_2d_dijkstra_bool.resize(plot_size,vector_1d_bool);
                long size;
                for (int i=0; i<get_map_stack.cost.size(); i++)
                {
                    vector_2d[zero_point + gridding.float_to_int(get_map_stack.cost[i].x)][zero_point + gridding.float_to_int(get_map_stack.cost[i].y)] = 1;
                }
                for (int i=0; i<get_map_stack.clearly.size(); i++)
                {
                    vector_2d[zero_point + gridding.float_to_int(get_map_stack.clearly[i].x)][zero_point + gridding.float_to_int(get_map_stack.clearly[i].y)] = 0;
                }
                size = get_map_stack.clearly.size();
                pose.position.x = robot_position.robot_pose.position.x;
                pose.position.y = robot_position.robot_pose.position.y;
                pose.position.z = 0.0;
                pose.orientation.w = 1.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.0;
                dijkstra_start_x = zero_point + gridding.float_to_int(gridding.float_to_grid(pose.position.x));
                dijkstra_start_y = zero_point + gridding.float_to_int(gridding.float_to_grid(pose.position.y));
                vector_2d_dijkstra_cost[dijkstra_start_x][dijkstra_start_y] = 0;
                vector_2d[dijkstra_start_x][dijkstra_start_y] = 0;
                vector_2d[dijkstra_start_x - 1][dijkstra_start_y - 1] = 0;
                vector_2d[dijkstra_start_x + 1][dijkstra_start_y + 1] = 0;
                vector_2d[dijkstra_start_x + 1][dijkstra_start_y - 1] = 0;
                vector_2d[dijkstra_start_x - 1][dijkstra_start_y + 1] = 0;
                vector_2d[dijkstra_start_x - 1][dijkstra_start_y] = 0;
                vector_2d[dijkstra_start_x + 1][dijkstra_start_y] = 0;
                vector_2d[dijkstra_start_x][dijkstra_start_y - 1] = 0;
                vector_2d[dijkstra_start_x][dijkstra_start_y + 1] = 0;
                limit_point_dijkstra[0] = dijkstra_start_x;
                limit_point_dijkstra[1] = dijkstra_start_x;
                limit_point_dijkstra[2] = dijkstra_start_y; 
                limit_point_dijkstra[3] = dijkstra_start_y;
                if (vector_2d_dijkstra_bool[dijkstra_start_x][dijkstra_start_y])
                {
                    printf("\n%ld\n",expansion_pose.poses.size());
                }
                while (ros::ok())
                {
                    min_cost = max_float;
                    for (int i=limit_point_dijkstra[0]; i<=limit_point_dijkstra[1]; i++)
                    {
                        for (int j=limit_point_dijkstra[2]; j<=limit_point_dijkstra[3]; j++)
                        {
                            if ((vector_2d_dijkstra_cost[i][j] <= min_cost) && (vector_2d_dijkstra_bool[i][j] == false) && (vector_2d[i][j] == 0))
                            {
                                min_cost = vector_2d_dijkstra_cost[i][j];
                                min_x = i;
                                min_y = j;
                            }
                        }
                    }
                    if ((min_x - 1) < limit_point_dijkstra[0])
                    {
                        limit_point_dijkstra[0] = (min_x - 1);
                    }
                    if (limit_point_dijkstra[1] < (min_x + 1))
                    {
                        limit_point_dijkstra[1] = (min_x + 1);
                    }
                    if ((min_y - 1) < limit_point_dijkstra[2])
                    {
                        limit_point_dijkstra[2] = (min_y - 1);
                    }
                    if (limit_point_dijkstra[3] < (min_y + 1))
                    {
                        limit_point_dijkstra[3] = (min_y + 1);
                    }
                    if (vector_2d_dijkstra_bool[min_x][min_y])
                    {
                        ros::spinOnce();
                        break;
                    }
                    else
                    {
                        vector_2d_dijkstra_bool[min_x][min_y] = true;
                    }
                    for (int i=min_x-1; i<=min_x+1; i++)
                    {
                        for (int j=min_y-1; j<=min_y+1; j++)
                        {
                            if (((i == min_x) && (j == min_y)) || (vector_2d_dijkstra_bool[i][j] == true) || (vector_2d[i][j] == 1) || (vector_2d[i][j] == -1))
                            {
                                if (vector_2d[i][j] == -1)
                                {
                                    geometry_msgs::Vector3 vec;
                                    vec.x = gridding.int_to_grid(min_x - zero_point);
                                    vec.y = gridding.int_to_grid(min_y - zero_point);
                                    vec.z = 0.0;
                                    expansion_pose.poses.push_back(vec);
                                    ros::spinOnce();
                                }
                            }
                            else if ((i != min_x) && (j != min_y))
                            {
                                if ((min_cost + sqrt(2)) < vector_2d_dijkstra_cost[i][j])
                                {
                                    vector_2d_dijkstra_cost[i][j] = min_cost + sqrt(2);
                                }
                            }
                            else
                            {
                                if ((min_cost + 1) < vector_2d_dijkstra_cost[i][j])
                                {
                                    vector_2d_dijkstra_cost[i][j] = min_cost + 1;
                                }
                            }
                        }
                    }
                }
                ros::spinOnce();
                pub_expansion.publish(expansion_pose);
                ros::spinOnce();
            }
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "expansion_pointer");
    EXPANSION_POINTER expansion_pointer;
    ros::spin();
}