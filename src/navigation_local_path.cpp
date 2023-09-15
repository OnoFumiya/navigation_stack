#include <stdio.h>
#include <ros/ros.h>
#include <unistd.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <limits>
#include <math.h>
#include <cmath>
#include <typeinfo>
#include <sys/time.h>
#include <matplotlib-cpp/matplotlibcpp.h>
#include <navigation_stack/MapInformation.h>
#include <navigation_stack/ExpansionPoints.h>
#include <navigation_stack/PathPoint.h>


#include <cstring>
#include <algorithm>


class PARAM
{
    public:
        // DWA

        // speed
        float sum_max_speed = 0.60;
        float sum_min_speed = 0.05;
        float max_speed_x = 0.30;
        float min_speed_x = 0.00;
        float max_speed_y = 0.00;
        float min_speed_y =-0.00;
        float max_angle = 100.0 * (M_PI/180.);

        // accel
        float max_accel_x = 0.50;
        float max_accel_y = 0.00;
        float max_angle_accel = 100.0 * (M_PI/180.);

        // weight
        float velocity_weight = 1.0;
        float angle_weight = 1.0;
        float obstacle_distance_weight = 1.0;

        // time
        float delta_time = 0.15;
        int predect_step = 4;

        // reso
        float speed_reso_x = 0.02;
        float speed_reso_y = 0.02;
        float speed_reso_ang = 5.0 * (M_PI/180.);

        // other
        float robot_radius = 0.4;
        float goal_position_range = 0.2;
        float local_position_range = 0.5;
        float goal_angle_range = 5.0 * (M_PI/180.);
        bool omni_base = false;
        int local_goal_range = 3;
};


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
        PARAM param;
        bool start_frag;
        void callback_obstacle(const sensor_msgs::LaserScan &ob)
        {
            ob_theta.clear();
            range_point.clear();
            range_angle_increment = ob.angle_increment;
            for (int i=0; i<ob.ranges.size(); i++)
            {
                if ((ob.range_min <= ob.ranges[i]) && (ob.ranges[i] <= ob.range_max))
                {
                    if (ob.ranges[i] <= ((param.sum_max_speed*param.delta_time*param.predect_step) + 2*param.obstacle_distance_weight*(param.robot_radius)))
                    {
                        point.x = robot_position.robot_pose.position.x + (ob.ranges[i]*(cos(robot_position.robot_theta + ob.angle_min + range_angle_increment*i))) + (lidar_pose[0]*cos(robot_position.robot_theta) - lidar_pose[1]*sin(robot_position.robot_theta));
                        point.y = robot_position.robot_pose.position.y + (ob.ranges[i]*(sin(robot_position.robot_theta + ob.angle_min + range_angle_increment*i))) + (lidar_pose[0]*sin(robot_position.robot_theta) + lidar_pose[1]*cos(robot_position.robot_theta));
                        point.z = 0.02;
                        ob_theta.push_back(robot_position.robot_theta + ob.angle_min + range_angle_increment*i);
                        range_point.push_back(point);
                    }
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


class MOVE_CLASS
{
    private:
        ROBOT_POSITION robot_position;
        ros::Publisher pub_twist;
    public:
        MOVE_CLASS()
        {
            ros::NodeHandle node;
            // pub_twist = node.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 10);
            pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
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
            vel.linear.x = 0.;
            vel.linear.y = 0.;
            vel.angular.z = 0.;
            ros::spinOnce();
            only_vel_pub(vel);
            ros::spinOnce();
            // struct timeval t;
            // gettimeofday(&t, NULL);    //時間を取得
            // long sec_0 = t.tv_sec, sec;     //整数部分の秒(s)
            // while (ros::ok())
            // {
            //     gettimeofday(&t, NULL);    //時間を取得
            //     sec = t.tv_sec;     //整数部分の秒(s)
            //     ros::spinOnce();
            //     pub_twist.publish(vel);
            //     if ((sec-sec_0) > 1.0)
            //     {
            //         break;
            //     }
            // }
        }
        void go(float st, float si, float t)
        {
            ros::spinOnce();
            straight_and_turn_time(st, si, t);
            stop_vel();
            ros::spinOnce();
        }
        void only_vel_pub(geometry_msgs::Twist vel)
        {
            pub_twist.publish(vel);
            ros::spinOnce();
        }
        void turn(float angle)
        {
            PARAM param;
            geometry_msgs::Twist vel;
            vel.linear.x = 0.;
            vel.linear.y = 0.;
            vel.linear.z = 0.;
            vel.angular.x = 0.;
            vel.angular.y = 0.;
            vel.angular.z = param.max_angle_accel * param.delta_time * (angle/std::fabs(angle));
            float last_angle = robot_position.odom_theta + angle;
            while (ros::ok())
            {
                if (std::fabs(last_angle - robot_position.odom_theta) <= 0.1)
                {
                    stop_vel();
                    break;
                }
                ros::spinOnce();
                only_vel_pub(vel);
                ros::spinOnce();
            }
        }
};


struct TRAJECTORY_NODE
{
    geometry_msgs::Point node;
    float angle;
};

class PATH_MOVING
{
    private:
        ROBOT_POSITION robot_position;
        OBSTACLE_DIST obstacle_dist;
        PARAM param;
        ros::Publisher pub_local_path;
        ros::Publisher pub_goal_flag;
        ros::Subscriber sub_globalpath;
        navigation_stack::PathPoint global_path;
        std_msgs::Bool end_flag;
        geometry_msgs::Pose goal_pose;
        std::vector<float> target_angle;
        int id_stack;
        void callback_globalpath(const navigation_stack::PathPoint &get_path)
        {
            global_path.poses.clear();
            target_angle.clear();
            geometry_msgs::Vector3 vec;
            vec.z = 0.;
            for (int i=0; i<get_path.poses.size(); i++)
            {
                if (i!=0)
                {
                    if (param.omni_base)
                    {
                        target_angle.push_back(atan2((get_path.poses[i].y - get_path.poses[i-1].y), (get_path.poses[i].x - get_path.poses[i-1].x)));
                    }
                }
                vec.x = get_path.poses[i].x;
                vec.y = get_path.poses[i].y;
                global_path.poses.push_back(vec);
            }
            target_angle.push_back((2*(acos(get_path.goal_pose.orientation.w)))*((get_path.goal_pose.orientation.z)*(get_path.goal_pose.orientation.w))/(std::fabs((get_path.goal_pose.orientation.z)*(get_path.goal_pose.orientation.w))));
            global_path.id = get_path.id;
            end_flag.data = false;
        }
    public:
        PATH_MOVING()
        {
            ros::NodeHandle node;
            pub_local_path = node.advertise<navigation_stack::PathPoint>("/local_path_planning", 10);
            pub_goal_flag = node.advertise<std_msgs::Bool>("/goal_flag", 10);
            sub_globalpath = node.subscribe("/global_path_planning", 10, &PATH_MOVING::callback_globalpath, this);
            move_control();
        }
        void move_control()
        {
            ros::spinOnce();
            end_flag.data = true;
            while (ros::ok())
            {
                ros::spinOnce();
                if (end_flag.data == true)
                {
                    ros::spinOnce();
                    continue;
                }
                else
                {
                    ros::spinOnce();
                    id_stack = -1;
                    path_plan();
                }
            }
            ros::spinOnce();
        }
        void path_plan()
        {
            MOVE_CLASS move_class;
            std::vector<geometry_msgs::Vector3> global_path_stack;
            std::vector<float> global_angle_stack;
            geometry_msgs::Vector3 vec;
            geometry_msgs::Twist vel;
            navigation_stack::PathPoint local_path;
            vel.linear.x = 0.;
            vel.linear.y = 0.;
            vel.linear.z = 0.;
            vel.angular.x = 0.;
            vel.angular.y = 0.;
            vel.angular.z = 0.;
            double dw[3][2] = {{vel.linear.x, vel.linear.x}, {vel.linear.y, vel.linear.y}, {vel.angular.z, vel.angular.z}};
            // geometry_msgs::Point robot_point;
            // float angle;
            int target_index = 0;
            while (ros::ok())
            {
                bool goal_flag = false;
                // robot_point.x = robot_position.robot_pose.position.x;
                // robot_point.y = robot_position.robot_pose.position.y;
                // robot_point.z = robot_position.robot_pose.position.z;
                // angle = robot_position.robot_theta;
                if (id_stack != -1)
                {
                    // if (euclidean_distance(robot_point.x, robot_point.y, global_path_stack[global_path_stack.size()-1].x, global_path_stack[global_path_stack.size()-1].y) <= param.goal_position_range)
                    if (euclidean_distance(robot_position.robot_pose.position.x, robot_position.robot_pose.position.y, global_path_stack[global_path_stack.size()-1].x, global_path_stack[global_path_stack.size()-1].y) <= param.goal_position_range)
                    {
                        goal_flag = true;
                    }
                }
                if (goal_flag)
                {
                    ROS_INFO("Goal Reached...next turn only...\n");
                    end_flag.data = true;
                    pub_goal_flag.publish(end_flag);
                    break;
                }
                float min_dist = std::numeric_limits<float>::max();
                int start_index;
                if (id_stack != global_path.id)
                {
                    ROS_INFO("id change\n");
                    id_stack = global_path.id;
                    global_path_stack.clear();
                    global_angle_stack.clear();
                    global_path_stack.resize(global_path.poses.size());
                    global_angle_stack.resize(target_angle.size());
                    copy(global_path.poses.begin(), global_path.poses.end(), global_path_stack.begin());
                    copy(target_angle.begin(), target_angle.end(), global_angle_stack.begin());
                    start_index = -1;
                    for (int i=0; i<global_path_stack.size(); i++)
                    {
                        // if (euclidean_distance(robot_point.x, robot_point.y, global_path_stack[i].x, global_path_stack[i].y) < min_dist)
                        if (euclidean_distance(robot_position.robot_pose.position.x, robot_position.robot_pose.position.y, global_path_stack[i].x, global_path_stack[i].y) < min_dist)
                        {
                            // min_dist = euclidean_distance(robot_point.x, robot_point.y, global_path_stack[i].x, global_path_stack[i].y);
                            min_dist = euclidean_distance(robot_position.robot_pose.position.x, robot_position.robot_pose.position.y, global_path_stack[i].x, global_path_stack[i].y);
                            start_index = i;
                        }
                    }
                    if (start_index == -1)
                    {
                        ROS_ERROR("MOVE ERROR\n");
                        vel.linear.x = 0.;
                        vel.linear.y = 0.;
                        vel.linear.z = 0.;
                        vel.angular.x = 0.;
                        vel.angular.y = 0.;
                        vel.angular.z = 0.;
                        move_class.stop_vel();
                        // return;
                        continue;
                    }
                    target_index = start_index + 1;
                }
                // else if (euclidean_distance(robot_point.x, robot_point.y, global_path_stack[target_index].x, global_path_stack[target_index].y) <= param.local_position_range)
                else if (euclidean_distance(robot_position.robot_pose.position.x, robot_position.robot_pose.position.y, global_path_stack[target_index].x, global_path_stack[target_index].y) <= param.local_position_range)
                {
                    if ((target_index + param.local_goal_range) <= (global_path_stack.size()-1))
                    {
                        ROS_INFO("target change\n");
                        target_index += param.local_goal_range;
                    }
                    else
                    {
                        target_index = global_path_stack.size()-1;
                    }
                }
                // int target_index = start_index + 1;
                if (global_path_stack.size() <= target_index)
                {
                    ROS_ERROR("NO PATH\n");
                    vel.linear.x = 0.;
                    vel.linear.y = 0.;
                    vel.linear.z = 0.;
                    vel.angular.x = 0.;
                    vel.angular.y = 0.;
                    vel.angular.z = 0.;
                    move_class.stop_vel();
                    // return;
                    continue;
                }
                dynamic_window(dw, vel);
                std::vector<std::vector<TRAJECTORY_NODE>> path_cans;
                std::vector<TRAJECTORY_NODE> path_can;
                std::vector<float> local_costs;
                std::vector<geometry_msgs::Twist> velocitys;
                geometry_msgs::Twist velocity;
                path_cans.clear();
                local_costs.clear();
                velocitys.clear();
                // robot_point.x = robot_position.robot_pose.position.x;
                // robot_point.y = robot_position.robot_pose.position.y;
                // robot_point.z = robot_position.robot_pose.position.z;
                // angle = robot_position.robot_theta;
                velocity.linear.x = 0.;
                velocity.linear.y = 0.;
                velocity.linear.z = 0.;
                velocity.angular.x = 0.;
                velocity.angular.y = 0.;
                velocity.angular.z = 0.;
                // ROS_INFO("dw = [[%.2f, %.2f], [%.2f, %.2f], [%.2f, %.2f]]\n",dw[0][1],dw[0][1],dw[1][0],dw[1][1],dw[1][1]);
                for (velocity.linear.x=dw[0][0]; velocity.linear.x<=dw[0][1]; velocity.linear.x+=param.speed_reso_x)
                {
                    for (velocity.linear.y=dw[1][0]; velocity.linear.y<=dw[1][1]; velocity.linear.y+=param.speed_reso_y)
                    {
                        for (velocity.angular.z=dw[2][0]; velocity.angular.z<=dw[2][1]; velocity.angular.z+=param.speed_reso_ang)
                        {
                            path_can.clear();
                            float local_cost;
                            // local_cost = sim_motion(robot_point, angle, path_can, velocity);
                            local_cost = sim_motion(robot_position.robot_pose.position, robot_position.robot_theta, path_can, velocity);
                            // ROS_INFO("velocity = %.2f, %.2f, %.2f\npath = \n[%.2f, %.2f], [%.2f, %.2f], [%.2f, %.2f], [%.2f, %.2f]\n\n",velocity.linear.x,velocity.linear.y,velocity.angular.z,path_can[0].node.x,path_can[0].node.y,path_can[1].node.x,path_can[1].node.y,path_can[2].node.x,path_can[2].node.y,path_can[3].node.x,path_can[3].node.y);
                            if (param.robot_radius <= local_cost)
                            {
                                if (((param.sum_max_speed*param.delta_time*param.predect_step) + 2*(param.robot_radius)) < local_cost)
                                {
                                    local_cost = ((param.sum_max_speed*param.delta_time*param.predect_step) + 2*(param.robot_radius));
                                }
                                path_cans.push_back(path_can);
                                local_costs.push_back(local_cost);
                                velocitys.push_back(velocity);
                            }
                        }
                    }
                }
                float sum_cost = std::numeric_limits<float>::max();
                int best_index;
                vel.linear.x = 0.;
                vel.linear.y = 0.;
                vel.linear.z = 0.;
                vel.angular.x = 0.;
                vel.angular.y = 0.;
                vel.angular.z = 0.;
                for (int i=0; i<path_cans.size(); i++)
                {
                    float cost_vel = (param.velocity_weight) * (param.sum_max_speed - sqrtf(powf(velocitys[i].linear.x, 2.) + powf(velocitys[i].linear.y, 2.))) / (param.sum_max_speed);
                    float cost_ang;
                    if (param.omni_base)
                    {
                        cost_ang = (param.angle_weight) * std::fabs(target_angle[target_index] - (robot_position.robot_theta+velocitys[i].angular.z*param.delta_time)) / (M_PI);
                    }
                    else
                    {
                        // cost_ang = (param.angle_weight) * atan2(global_path_stack[target_index].y - robot_point.y, global_path_stack[target_index].x - robot_point.x);
                        cost_ang = (param.angle_weight) * atan2(global_path_stack[target_index].y - robot_position.robot_pose.position.y, global_path_stack[target_index].x - robot_position.robot_pose.position.x) / (M_PI);
                    }
                    float cost_dist = (param.obstacle_distance_weight) * ((param.sum_max_speed*param.delta_time*param.predect_step) + 2*(param.robot_radius)) / local_costs[i];
                    if ((cost_vel + cost_ang + cost_dist) <= sum_cost)
                    {
                        sum_cost = cost_vel + cost_ang + cost_dist;
                        vel.linear.x = velocitys[i].linear.x;
                        vel.linear.y = velocitys[i].linear.y;
                        vel.angular.z = velocitys[i].angular.z;
                        best_index = i;
                    }
                }
                local_path.poses.clear();
                for (int i=0; i<path_cans[best_index].size(); i++)
                {
                    geometry_msgs::Vector3 pt;
                    pt.x = path_cans[best_index][i].node.x;
                    pt.y = path_cans[best_index][i].node.y;
                    pt.z = 0.;
                    local_path.poses.push_back(pt);
                }
                pub_local_path.publish(local_path);
                ros::spinOnce();
                // ROS_INFO("vel = %.2f ,%.2f ,%.2f\n",vel.linear.x,vel.linear.x,vel.angular.z);
                move_class.only_vel_pub(vel);
                ros::Duration(param.delta_time).sleep();
            }
            ros::spinOnce();
            while (ros::ok())
            {
                ros::spinOnce();
                if ((global_angle_stack[global_angle_stack.size() - 1] - robot_position.robot_theta) <= param.goal_angle_range)
                {
                    break;
                }
                else
                {
                    ros::spinOnce();
                    move_class.turn(global_angle_stack[global_angle_stack.size()-1]-robot_position.robot_theta);
                }
            }
        }
        void dynamic_window(double dw[3][2], geometry_msgs::Twist vel)
        {
            double dw1[3][2] = {{param.min_speed_x                                , param.max_speed_x                                },{param.min_speed_y                                , param.max_speed_y                                },{(param.max_angle)*(-1)                                , param.max_angle                                       }};
            double dw2[3][2] = {{vel.linear.x - param.max_accel_x*param.delta_time, vel.linear.x + param.max_accel_x*param.delta_time},{vel.linear.y - param.max_accel_y*param.delta_time, vel.linear.y + param.max_accel_y*param.delta_time},{vel.angular.z - param.max_angle_accel*param.delta_time, vel.angular.z + param.max_angle_accel*param.delta_time}};
            for (int i=0; i<3; i++)
            {
                for (int j=0; j<2; j++)
                {
                    if (((dw1[i][j] < dw2[i][j]) && (j==0)) || ((dw2[i][j] < dw1[i][j]) && (j==1)))
                    {
                        dw[i][j] = dw2[i][j];
                    }
                    else
                    {
                        dw[i][j] = dw1[i][j];
                    }
                }
            }
            // ROS_INFO("vel = %.2f, %.2f, %.2f",vel.linear.x,vel.linear.y,vel.angular.z);
            // ROS_INFO("dw1 = [[%.2f, %.2f], [%.2f, %.2f], [%.2f, %.2f]]",dw1[0][0],dw1[0][1],dw1[1][0],dw1[1][1],dw1[2][0],dw1[2][1]);
            // ROS_INFO("dw2 = [[%.2f, %.2f], [%.2f, %.2f], [%.2f, %.2f]]",dw2[0][0],dw2[0][1],dw2[1][0],dw2[1][1],dw2[2][0],dw2[2][1]);
            // ROS_INFO("dw  = [[%.2f, %.2f], [%.2f, %.2f], [%.2f, %.2f]]\n",dw[0][0],dw[0][1],dw[1][0],dw[1][1],dw[2][0],dw[2][1]);
        }
        float sim_motion(geometry_msgs::Point robot_pt, float robot_angle, std::vector<TRAJECTORY_NODE> &path_can, geometry_msgs::Twist v)
        {
            TRAJECTORY_NODE pt;
            pt.node.x = robot_pt.x;
            pt.node.y = robot_pt.y;
            pt.node.z = 0.;
            pt.angle = robot_angle;
            float min_cost = std::numeric_limits<float>::max();
            float theta = robot_angle + atan2(v.linear.y * param.delta_time, v.linear.x * param.delta_time);
            if (v.angular.z == 0.)
            {
                for (int i=0; i<param.predect_step; i++)
                {
                    pt.node.x += sqrtf(powf(v.linear.x * param.delta_time, 2.) + powf(v.linear.y * param.delta_time, 2.)) * cos(theta);
                    pt.node.y += sqrtf(powf(v.linear.x * param.delta_time, 2.) + powf(v.linear.y * param.delta_time, 2.)) * sin(theta);
                    path_can.push_back(pt);
                    for (int j=0; j<obstacle_dist.range_point.size(); j++)
                    {
                        if (euclidean_distance(pt.node.x, pt.node.y, obstacle_dist.range_point[j].x, obstacle_dist.range_point[j].y) < min_cost)//param.robot_radius)
                        {
                            min_cost = euclidean_distance(pt.node.x, pt.node.y, obstacle_dist.range_point[j].x, obstacle_dist.range_point[j].y);
                        }
                    }
                }
            }
            else if(sqrtf(powf(v.linear.x, 2.) + powf(v.linear.y, 2.)) == 0.0)
            {
                for (int i=0; i<param.predect_step; i++)
                {
                    pt.angle += v.angular.z * param.delta_time;
                    path_can.push_back(pt);
                }
            }
            else
            {
                float base_angle = robot_angle + atan2(v.linear.y * param.delta_time, v.linear.x * param.delta_time) + (M_PI / 2.)*((v.angular.z * param.delta_time) / std::fabs(v.angular.z * param.delta_time));
                float circle_radius = (sqrtf(powf(v.linear.x * param.delta_time, 2.) + powf(v.linear.y * param.delta_time, 2.))) / std::fabs(v.angular.z * param.delta_time);
                geometry_msgs::Point circle_pt;
                circle_pt.x = robot_pt.x + circle_radius * cos(base_angle);
                circle_pt.y = robot_pt.y + circle_radius * sin(base_angle);
                for (int i=0; i<param.predect_step; i++)
                {
                    float temp_x = pt.node.x;
                    float temp_y = pt.node.y;
                    pt.node.x = (temp_x - circle_pt.x) * cos(v.angular.z * param.delta_time) - (temp_y - circle_pt.y) * sin(v.angular.z * param.delta_time) + circle_pt.x;
                    pt.node.y = (temp_x - circle_pt.x) * sin(v.angular.z * param.delta_time) + (temp_y - circle_pt.y) * cos(v.angular.z * param.delta_time) + circle_pt.y;
                    pt.angle += v.angular.z * param.delta_time;
                    path_can.push_back(pt);
                    // ROS_INFO("pt = %.2f, %.2f\n",pt.node.x, pt.node.y);
                    for (int j=0; j<obstacle_dist.range_point.size(); j++)
                    {
                        if (euclidean_distance(pt.node.x, pt.node.y, obstacle_dist.range_point[j].x, obstacle_dist.range_point[j].y) < min_cost)
                        {
                            min_cost = euclidean_distance(pt.node.x, pt.node.y, obstacle_dist.range_point[j].x, obstacle_dist.range_point[j].y);
                        }
                    }
                }
            }
            return min_cost;
        }
        float euclidean_distance(float x0, float y0, float x1, float y1)
        {
            return (sqrtf(powf((x1 - x0), 2.) + powf((y1 - y0), 2.)));
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_local_path");
    PATH_MOVING path_moving;
    ros::spin();
}
