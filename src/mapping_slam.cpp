#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <iostream>
#include <limits>
#include <math.h>
#include <cmath>
#include <typeinfo>
#include <sys/time.h>
#include <matplotlib-cpp/matplotlibcpp.h>
#include <navigation_stack/MapInformation.h>
#include <geometry_msgs/Pose.h>



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
            return (float)((s/arg_size) + (1/(2*arg_size)));
        }
};


class ROBOT_POSITION
{
    private:
        ros::Subscriber sub_odom;
        void callback_odom(const nav_msgs::Odometry &odom)
        {
            position_x = odom.pose.pose.position.x + missed.position.x;
            position_y = odom.pose.pose.position.y + missed.position.y;
            position_z = odom.pose.pose.position.z + missed.position.z;
            sita = (2*(acos(odom.pose.pose.orientation.w + missed.orientation.w)))*((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w))/(std::fabs((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w)));
            if (std::isnan(sita) == true)
            {
                sita = 0.0;
            }
            if ((std::fabs(sita)) > M_PI)
            {
                sita = (2*M_PI - std::fabs(sita))*(((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w))/(std::fabs((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w))));
            }
            robot_position_frag = true;
        }
    public:
        geometry_msgs::Pose missed;
        float position_x, position_y, position_z, sita;
        bool robot_position_frag;
        ROBOT_POSITION()
        {
            ros::NodeHandle node;
            robot_position_frag = false;
            sub_odom = node.subscribe("/odom", 10, &ROBOT_POSITION::callback_odom, this);
            get_point();
        }
        void get_point()
        {
            position_x = 0.0;
            position_y = 0.0;
            position_z = 0.0;
            sita = 0.0;
            robot_position_frag = false;
            ros::spinOnce();
            while (ros::ok())
            {
                ros::spinOnce();
                if (robot_position_frag)
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
        ROBOT_POSITION robot_position;
        float maxdist, mindist;
        int i;
        void callback_obstacle(const sensor_msgs::LaserScan &ob)
        {
            range.clear();
            range_point.clear();
            float a;
            range_angle_increment = ob.angle_increment;
            maxdist = ob.range_max;
            mindist = ob.range_min;
            for (i=0; i<ob.ranges.size(); i++)
            {
                if ((mindist <= ob.ranges[i]) && (ob.ranges[i] <= maxdist))
                {
                    a = (robot_position.sita + ob.angle_min + range_angle_increment*i);
                    range.push_back(ob.ranges[i]);
                    range_point.push_back({robot_position.position_x + (ob.ranges[i]*(cos(a))), robot_position.position_y + (ob.ranges[i]*(sin(a))), 0, a});
                    // if (((std::isnan(range_point[i][0])) != true) && ((std::isnan(range_point[i][1])) != true))
                    // {
                    //     start_frag = true;
                    // }
                }
            }
            start_frag = true;
        }
    public:
        std::vector<float> range;
        std::vector<std::vector<float>> range_point;
        bool start_frag;
        float range_angle_increment;
        OBSTACLE_DIST()
        {
            ros::NodeHandle node;
            sub_dist = node.subscribe("/scan", 10, &OBSTACLE_DIST::callback_obstacle, this);
            robot_position.missed.position.x = 0.0;
            robot_position.missed.position.y = 0.0;
            robot_position.missed.position.z = 0.0;
            robot_position.missed.orientation.w = 0.0;
            robot_position.missed.orientation.x = 0.0;
            robot_position.missed.orientation.y = 0.0;
            robot_position.missed.orientation.z = 0.0;
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
                ros::spinOnce();
            }
            ros::spinOnce();
        }
};


class SLAM
{
    private:
        ros::Publisher pub_map;
        GRIDDING gridding;
        ROBOT_POSITION robot_position;
        OBSTACLE_DIST obstacle_dist;
        navigation_stack::MapInformation map;
        int plot_size;
        int zero_point;
        int limit_point[4];
        std::vector<int> vector_1d;
        std::vector<std::vector<int>> vector_2d;
    public:
        SLAM()
        {
            ros::NodeHandle node;
            pub_map = node.advertise<navigation_stack::MapInformation>("/mapping", 10);
            plot_size = (int)((sqrt(std::numeric_limits<int>::max()))/3);
            zero_point = (int)(plot_size/2);
            vector_1d.resize(plot_size,-1);
            vector_2d.resize(plot_size,vector_1d);
            SLAM_control();
        }
        void SLAM_control()
        {
            limit_point[0] = std::numeric_limits<int>::max();
            limit_point[1] = (std::numeric_limits<int>::max())*(-1);
            limit_point[2] = std::numeric_limits<int>::max();
            limit_point[3] = (std::numeric_limits<int>::max())*(-1);
            ros::spinOnce();
            while (ros::ok())
            {
                ros::spinOnce();
                // localization(robot_position.missed);
                // ros::spinOnce();
                mapping();
                ros::spinOnce();
            }
        }
        // void localization(geometry_msgs::Pose &miss)
        // {
        // }
        void mapping()
        {
            geometry_msgs::Vector3 vec;
            vec.z = 0.0;
            for (int i=0; i<obstacle_dist.range_point.size(); i++)
            {
                ros::spinOnce();
                vector_2d[zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i][0]))][zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i][1]))] = 1;
                if ((zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i][0]))) < limit_point[0])
                {
                    limit_point[0] = zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i][0]));
                }
                if (limit_point[1] < (zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i][0]))))
                {
                    limit_point[1] = zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i][0]));
                }
                if ((zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i][1]))) < limit_point[2])
                {
                    limit_point[2] = zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i][1]));
                }
                if (limit_point[3] < (zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i][1]))))
                {
                    limit_point[3] = zero_point + gridding.float_to_int(gridding.float_to_grid(obstacle_dist.range_point[i][1]));
                }
                if ((((M_PI/4)) < std::fabs(obstacle_dist.range_point[i][3])) && (std::fabs(obstacle_dist.range_point[i][3]) < ((3*M_PI/4))))
                {
                    for (int j=1; j<((std::fabs(obstacle_dist.range_point[i][1] - robot_position.position_y))/gridding.size); j++)
                    {
                        if (vector_2d[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_x + ((j*gridding.size/tan(obstacle_dist.range_point[i][3]))*((std::fabs(obstacle_dist.range_point[i][1]-robot_position.position_y))/(obstacle_dist.range_point[i][1]-robot_position.position_y)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_y + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i][1]-robot_position.position_y))/(obstacle_dist.range_point[i][1]-robot_position.position_y)))))] != 1)
                        {
                            vector_2d[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_x + ((j*gridding.size/tan(obstacle_dist.range_point[i][3]))*((std::fabs(obstacle_dist.range_point[i][1]-robot_position.position_y))/(obstacle_dist.range_point[i][1]-robot_position.position_y)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_y + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i][1]-robot_position.position_y))/(obstacle_dist.range_point[i][1]-robot_position.position_y)))))] = 0;
                        }
                        if ((zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_x + ((j*gridding.size/tan(obstacle_dist.range_point[i][3]))*((std::fabs(obstacle_dist.range_point[i][1]-robot_position.position_y))/(obstacle_dist.range_point[i][1]-robot_position.position_y)))))) < limit_point[0])
                        {
                            limit_point[0] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_x + ((j*gridding.size/tan(obstacle_dist.range_point[i][3]))*((std::fabs(obstacle_dist.range_point[i][1]-robot_position.position_y))/(obstacle_dist.range_point[i][1]-robot_position.position_y)))));
                        }
                        if (limit_point[1] < (zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_x + ((j*gridding.size/tan(obstacle_dist.range_point[i][3]))*((std::fabs(obstacle_dist.range_point[i][1]-robot_position.position_y))/(obstacle_dist.range_point[i][1]-robot_position.position_y)))))))
                        {
                            limit_point[1] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_x + ((j*gridding.size/tan(obstacle_dist.range_point[i][3]))*((std::fabs(obstacle_dist.range_point[i][1]-robot_position.position_y))/(obstacle_dist.range_point[i][1]-robot_position.position_y)))));
                        }
                        if ((zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_y + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i][1]-robot_position.position_y))/(obstacle_dist.range_point[i][1]-robot_position.position_y)))))) < limit_point[2])
                        {
                            limit_point[2] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_y + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i][1]-robot_position.position_y))/(obstacle_dist.range_point[i][1]-robot_position.position_y)))));
                        }
                        if (limit_point[3] < (zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_y + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i][1]-robot_position.position_y))/(obstacle_dist.range_point[i][1]-robot_position.position_y)))))))
                        {
                            limit_point[3] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_y + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i][1]-robot_position.position_y))/(obstacle_dist.range_point[i][1]-robot_position.position_y)))));
                        }
                    }
                }
                else
                {
                    for (int j=1; j<((std::fabs(obstacle_dist.range_point[i][0] - robot_position.position_x))/gridding.size); j++)
                    {
                        // if (vector_2d[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_x + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i][0]-robot_position.position_x))/(obstacle_dist.range_point[i][0]-robot_position.position_x)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_y + ((j*gridding.size*tan(obstacle_dist.range_point[i][3]))*((std::fabs(obstacle_dist.range_point[i][0]-robot_position.position_x))/(obstacle_dist.range_point[i][0]-robot_position.position_x)))))] != 1)
                        // {
                        //     vector_2d[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_x + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i][0]-robot_position.position_x))/(obstacle_dist.range_point[i][0]-robot_position.position_x)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_y + ((j*gridding.size*tan(obstacle_dist.range_point[i][3]))*((std::fabs(obstacle_dist.range_point[i][0]-robot_position.position_x))/(obstacle_dist.range_point[i][0]-robot_position.position_x)))))] = 0;
                        // }
                        vector_2d[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_x + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i][0]-robot_position.position_x))/(obstacle_dist.range_point[i][0]-robot_position.position_x)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_y + ((j*gridding.size*tan(obstacle_dist.range_point[i][3]))*((std::fabs(obstacle_dist.range_point[i][0]-robot_position.position_x))/(obstacle_dist.range_point[i][0]-robot_position.position_x)))))] = 0;
                        if ((zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_x + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i][0]-robot_position.position_x))/(obstacle_dist.range_point[i][0]-robot_position.position_x)))))) < limit_point[0])
                        {
                            limit_point[0] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_x + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i][0]-robot_position.position_x))/(obstacle_dist.range_point[i][0]-robot_position.position_x)))));
                        }
                        if (limit_point[1] < (zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_x + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i][0]-robot_position.position_x))/(obstacle_dist.range_point[i][0]-robot_position.position_x)))))))
                        {
                            limit_point[1] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_x + ((j*gridding.size                                     )*((std::fabs(obstacle_dist.range_point[i][0]-robot_position.position_x))/(obstacle_dist.range_point[i][0]-robot_position.position_x)))));
                        }
                        if ((zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_y + ((j*gridding.size*tan(obstacle_dist.range_point[i][3]))*((std::fabs(obstacle_dist.range_point[i][0]-robot_position.position_x))/(obstacle_dist.range_point[i][0]-robot_position.position_x)))))) < limit_point[2])
                        {
                            limit_point[2] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_y + ((j*gridding.size*tan(obstacle_dist.range_point[i][3]))*((std::fabs(obstacle_dist.range_point[i][0]-robot_position.position_x))/(obstacle_dist.range_point[i][0]-robot_position.position_x)))));
                        }
                        if (limit_point[3] < (zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_y + ((j*gridding.size*tan(obstacle_dist.range_point[i][3]))*((std::fabs(obstacle_dist.range_point[i][0]-robot_position.position_x))/(obstacle_dist.range_point[i][0]-robot_position.position_x)))))))
                        {
                            limit_point[3] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position_y + ((j*gridding.size*tan(obstacle_dist.range_point[i][3]))*((std::fabs(obstacle_dist.range_point[i][0]-robot_position.position_x))/(obstacle_dist.range_point[i][0]-robot_position.position_x)))));
                        }
                    }
                }
            }
            for (int i=limit_point[0]; i<=limit_point[1]; i++)
            {
                for (int j=limit_point[2]; j<=limit_point[3]; j++)
                {
                    if ((vector_2d[i][j]==1) || (vector_2d[i][j]==0))
                    {
                        vec.x = gridding.int_to_grid(i-zero_point);
                        vec.y = gridding.int_to_grid(j-zero_point);
                        if (vector_2d[i][j] == 1)
                        {
                            map.cost.push_back(vec);
                        }
                        else
                        {
                            map.clearly.push_back(vec);
                        }
                    }
                }
            }
            ros::spinOnce();
            pub_map.publish(map);
            ros::spinOnce();
            map.cost.clear();
            map.clearly.clear();
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_slam");
    SLAM slam;
    ros::spin();
    return 0;
}