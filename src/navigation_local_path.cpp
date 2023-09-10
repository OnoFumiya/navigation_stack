#include <stdio.h>
#include <ros/ros.h>
#include <unistd.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
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
            // position_x = odom.pose.pose.position.x + missed.position.x;
            // position_y = odom.pose.pose.position.y + missed.position.y;
            // position_z = odom.pose.pose.position.z + missed.position.z;
            // theta = (2*(acos(odom.pose.pose.orientation.w + missed.orientation.w)))*((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w))/(std::fabs((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w)));
            // if ((std::fabs(theta)) > M_PI)
            // {
            //     theta = (2*M_PI - std::fabs(theta))*(((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w))/(std::fabs((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w))));
            // }
        }
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
        geometry_msgs::Point odom_pose;
        float robot_theta = 0.0;
        float odom_theta = 0.0;
        geometry_msgs::Point odom_pose_stack;
        float odom_theta_stack = 0.0;
        // geometry_msgs::Pose missed;
        // float position_x = 100.0, position_y = 100.0, position_z = 0.0, theta;
        float theta;
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
                if ((sec-sec_0) > 1.0)
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


class PATH_PLANNING
{
    private:
        OBSTACLE_DIST obstacle_dist;
        ros::Publisher pub_local_path;
        ros::Subscriber sub_map;
        navigation_stack::MapInformation map;
        bool map_set_frag;
        geometry_msgs::Pose goal_pose;
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
    public:
        PATH_PLANNING()
        {
            ros::NodeHandle node;
            pub_local_path = node.advertise<navigation_stack::MapInformation>("/local_path_planning", 10);
            sub_map = node.subscribe("/mapping", 10, &PATH_PLANNING::callback_map, this);
        }
        void wait_map()
        {
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
        void path_plan()
        {
            goal_pose.position.x = 2;
            goal_pose.position.y = 3;
            goal_pose.position.z = 0.0;
            goal_pose.orientation.w = 1.0;
            goal_pose.orientation.x = 0.0;
            goal_pose.orientation.y = 0.0;
            goal_pose.orientation.z = 0.0;
            std::vector<geometry_msgs::Point> global_path;
            bool goal_flag = false;
            while (ros::ok())
            {
                geometry_msgs::Point robot_point;
                geometry_msgs::Point goal_point;
                robot_point.x = obstacle_dist.robot_position.robot_pose.position.x;
                robot_point.y = obstacle_dist.robot_position.robot_pose.position.y;
                robot_point.z = obstacle_dist.robot_position.robot_pose.position.z;
                goal_flag = global_path_planning(global_path);
                ros::spinOnce();
                if (goal_flag)
                {
                    ros::spinOnce();
                    break;
                }
            }
        }
        bool global_path_planning(std::vector<geometry_msgs::Point> &global_path)
        {
            global_path.clear();
            return true;
        }
        // void 
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_local_path");
    PATH_PLANNING path_planning;
    ros::spin();
}