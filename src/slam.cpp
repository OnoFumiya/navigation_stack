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
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <limits>
#include <math.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <typeinfo>
#include <sys/time.h>
// #include <matplotlib-cpp/matplotlibcpp.h>
#include <navigation_stack/MapInformation.h>



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


// class ROBOT_POSITION
// {
//     public:
//         geometry_msgs::Point robot_pose;
//         float theta;
//         ROBOT_POSITION()
//         {
//             robot_pose.x = 0.0;
//             robot_pose.y = 0.0;
//             robot_pose.z = 0.0;
//             theta = 0.0;
//         }
// };


// class ODOM_POSITION
// {
//     private:
//         ros::Subscriber sub_odom;
//         void callback_odom(const nav_msgs::Odometry &odom)
//         {
//             odom_pose.x = odom.pose.pose.position.x;
//             odom_pose.y = odom.pose.pose.position.y;
//             odom_pose.z = odom.pose.pose.position.z;
//             theta = (2*(acos(odom.pose.pose.orientation.w)))*((odom.pose.pose.orientation.z)*(odom.pose.pose.orientation.w))/(std::fabs((odom.pose.pose.orientation.z)*(odom.pose.pose.orientation.w)));
//             if (std::isnan(theta) == true)
//             {
//                 theta = 0.0;
//             }
//             if ((std::fabs(theta)) > M_PI)
//             {
//                 theta = (2*M_PI - std::fabs(theta))*(((odom.pose.pose.orientation.z)*(odom.pose.pose.orientation.w))/(std::fabs((odom.pose.pose.orientation.z)*(odom.pose.pose.orientation.w))));
//             }
//             odom_position_frag = true;
//         }
//     public:
//         geometry_msgs::Point odom_pose;
//         float theta;
//         bool odom_position_frag;
//         ODOM_POSITION()
//         {
//             ros::NodeHandle node;
//             odom_position_frag = false;
//             sub_odom = node.subscribe("/odom", 10, &ODOM_POSITION::callback_odom, this);
//             get_point();
//         }
//         void get_point()
//         {
//             odom_pose.x = 0.0;
//             odom_pose.y = 0.0;
//             odom_pose.z = 0.0;
//             theta = 0.0;
//             odom_position_frag = false;
//             ros::spinOnce();
//             while (ros::ok())
//             {
//                 ros::spinOnce();
//                 if (odom_position_frag)
//                 {
//                     break;
//                 }
//             }
//         }
// };



class ROBOT_POSITION
{
    private:
        ros::Subscriber sub_odom;
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
        // float theta;
        ROBOT_POSITION()
        {
            ros::NodeHandle node;
            sub_odom = node.subscribe("/odom", 10, &ROBOT_POSITION::callback_odom, this);
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
                    robot_pose.position.x = odom_pose.x;
                    robot_pose.position.y = odom_pose.y;
                    robot_pose.position.z = odom_pose.z;
                    robot_theta = odom_theta;
                    robot_pose.orientation.w = cos(robot_theta / 2.);
                    robot_pose.orientation.x = 0.;
                    robot_pose.orientation.y = 0.;
                    robot_pose.orientation.z = sin(robot_theta / 2.);
                    break;
                }
            }
        }
        // void get_robot_pose()
        // {
        //     sub_robot = node.subscribe("/robot_position", 10, &ROBOT_POSITION::callback_robot, this);
        // }
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


// class OBSTACLE_DIST
// {
//     private:
//         ros::Subscriber sub_dist;
//         float maxdist, mindist;
//         float lidar_pose[2] = {0.0, 0.0};
//         void callback_obstacle(const sensor_msgs::LaserScan &ob)
//         {
//             geometry_msgs::Point point;
//             angle.clear();
//             range.clear();
//             range_point.clear();
//             range_angle_increment = ob.angle_increment;
//             maxdist = ob.range_max;
//             mindist = ob.range_min;
//             for (int i=0; i<ob.ranges.size(); i++)
//             {
//                 if ((mindist <= ob.ranges[i]) && (ob.ranges[i] <= maxdist))
//                 {
//                     angle.push_back(robot_position.theta + ob.angle_min + range_angle_increment*i);
//                     range.push_back(ob.ranges[i]);
//                     point.x = robot_position.robot_pose.x + (ob.ranges[i]*(cos(robot_position.theta + ob.angle_min + range_angle_increment*i))) + (lidar_pose[0]*cos(robot_position.theta) - lidar_pose[1]*sin(robot_position.theta));
//                     point.y = robot_position.robot_pose.y + (ob.ranges[i]*(sin(robot_position.theta + ob.angle_min + range_angle_increment*i))) + (lidar_pose[0]*sin(robot_position.theta) + lidar_pose[1]*cos(robot_position.theta));
//                     point.z = 0.0;
//                     range_point.push_back(point);
//                 }
//             }
//             start_frag = true;
//         }
//     public:
//         ROBOT_POSITION robot_position;
//         std::vector<float> angle;
//         std::vector<float> range;
//         std::vector<geometry_msgs::Point> range_point;
//         bool start_frag;
//         float range_angle_increment;
//         OBSTACLE_DIST()
//         {
//             ros::NodeHandle node;
//             sub_dist = node.subscribe("/scan", 10, &OBSTACLE_DIST::callback_obstacle, this);
//             get_dist();
//         }
//         void get_dist()
//         {
//             start_frag = false;
//             ros::spinOnce();
//             while(ros::ok())
//             {
//                 ros::spinOnce();
//                 if (start_frag)
//                 {
//                     ros::spinOnce();
//                     break;
//                 }
//                 ros::spinOnce();
//             }
//             ros::spinOnce();
//         }
// };


class SLAM
{
    private:
        ros::NodeHandle node_h;
        std::string mode;
        ros::Publisher pub_map;
        ros::Publisher pub_robot_position;
        ros::Subscriber sub_initial;
        GRIDDING gridding;
        // ODOM_POSITION odom_position;
        // ROBOT_POSITION robot_position;
        OBSTACLE_DIST obstacle_dist;
        // geometry_msgs::Point odom_stack_pose;.
        // float odom_stack_theta;
        navigation_stack::MapInformation map;
        geometry_msgs::Pose robot_position;
        int plot_size;
        int zero_point;
        int limit_point[4];
        float linear_x_missed_range = 0.20; // [m]
        float linear_y_missed_range = 0.20; // [m]
        float angular_missed_range = 0.17453292519943295;  // [rad]
        float angular_missed_grid = 0.03490658503988659; // [rad]
        std::vector<int> vector_1d;
        std::vector<std::vector<int>> vector_2d;
        std::string map_file_path;
        void callback_initial(const geometry_msgs::PoseWithCovarianceStamped &initial)
        {
            obstacle_dist.robot_position.robot_pose.position.x = initial.pose.pose.position.x;
            obstacle_dist.robot_position.robot_pose.position.y = initial.pose.pose.position.y;
            obstacle_dist.robot_position.robot_pose.position.z = initial.pose.pose.position.z;
            obstacle_dist.robot_position.robot_theta = (2*(acos(initial.pose.pose.orientation.w)))*((initial.pose.pose.orientation.z)*(initial.pose.pose.orientation.w))/(std::fabs((initial.pose.pose.orientation.z)*(initial.pose.pose.orientation.w)));
            obstacle_dist.robot_position.robot_pose.orientation.w = initial.pose.pose.orientation.w;
            obstacle_dist.robot_position.robot_pose.orientation.x = initial.pose.pose.orientation.x;
            obstacle_dist.robot_position.robot_pose.orientation.y = initial.pose.pose.orientation.y;
            obstacle_dist.robot_position.robot_pose.orientation.z = initial.pose.pose.orientation.z;
            // odom_pose_stack.x = 0.;
            // odom_pose_stack.y = 0.;
            // odom_pose_stack.z = 0.;
            // odom_theta_stack = 0.;
            obstacle_dist.robot_position.odom_pose_stack.x = obstacle_dist.robot_position.odom_pose.x;
            obstacle_dist.robot_position.odom_pose_stack.y = obstacle_dist.robot_position.odom_pose.y;
            obstacle_dist.robot_position.odom_pose_stack.z = obstacle_dist.robot_position.odom_pose.z;
            obstacle_dist.robot_position.odom_theta_stack = obstacle_dist.robot_position.odom_theta;
        }
    public:
        SLAM() : node_h("~")
        {
            ros::NodeHandle node;
            pub_map = node.advertise<navigation_stack::MapInformation>("/mapping", 10);
            pub_robot_position = node.advertise<geometry_msgs::Pose>("/robot_position", 10);
            plot_size = (int)((sqrt(std::numeric_limits<int>::max()))/4);
            zero_point = (int)(plot_size/2);
            vector_1d.resize(plot_size,-1);
            vector_2d.resize(plot_size,vector_1d);
            mode = node_h.param<std::string>( "mode", "NAVIGATION" );
            if (mode == "NAVIGATION")
            {
                map_file_path = node_h.param<std::string>( "map_file", "map_file.yaml" );
                sub_initial = node.subscribe("/initialpose", 10, &SLAM::callback_initial, this);
                get_map();
            }
            SLAM_control();
        }
        void SLAM_control()
        {
            limit_point[0] = std::numeric_limits<int>::max();
            limit_point[1] = (std::numeric_limits<int>::max())*(-1);
            limit_point[2] = std::numeric_limits<int>::max();
            limit_point[3] = (std::numeric_limits<int>::max())*(-1);
            ros::spinOnce();
            // if (mode == "NAVIGATION")
            // {
            //     // save_file = ros::package::getPath("navigation_stack") + "/map/";
            //     get_map();
            // }
            ros::spinOnce();
            float true_diff_x = 0., true_diff_y = 0., true_diff_theta = 0.;
            while (ros::ok())
            {
                ros::spinOnce();
                std::vector<geometry_msgs::Point> range_point_stack;
                std::vector<float> angle_stack;
                range_point_stack.resize(obstacle_dist.range_point.size());
                angle_stack.resize(obstacle_dist.ob_theta.size());
                copy(obstacle_dist.range_point.begin(), obstacle_dist.range_point.end(), range_point_stack.begin());
                copy(obstacle_dist.ob_theta.begin(), obstacle_dist.ob_theta.end(), angle_stack.begin());
                ros::spinOnce();
                localization(obstacle_dist.robot_position.robot_pose.position, range_point_stack, angle_stack, obstacle_dist.robot_position.odom_pose.x - obstacle_dist.robot_position.odom_pose_stack.x, obstacle_dist.robot_position.odom_pose.y - obstacle_dist.robot_position.odom_pose_stack.y, obstacle_dist.robot_position.odom_theta - obstacle_dist.robot_position.odom_theta_stack, true_diff_x, true_diff_y, true_diff_theta);
                // ROS_INFO("true_diff_x = %.2f",true_diff_x);
                obstacle_dist.robot_position.robot_pose.position.x += true_diff_x;
                obstacle_dist.robot_position.robot_pose.position.y += true_diff_y;
                obstacle_dist.robot_position.robot_theta += true_diff_theta;
                while (ros::ok())
                {
                    if (std::fabs(obstacle_dist.robot_position.robot_theta) > M_PI)
                    {
                        if (obstacle_dist.robot_position.robot_theta > 0.)
                        {
                            obstacle_dist.robot_position.robot_theta -= 2*M_PI;
                        }
                        else
                        {
                            obstacle_dist.robot_position.robot_theta += 2*M_PI;
                        }
                    }
                    if (std::fabs(obstacle_dist.robot_position.robot_theta) <= M_PI)
                    {
                        break;
                    }
                }
                // ROS_INFO("robot_ = %.2f, %.2f, %.2f\n",obstacle_dist.robot_position.robot_pose.x,obstacle_dist.robot_position.robot_pose.y,obstacle_dist.robot_position.theta);
                obstacle_dist.robot_position.odom_pose_stack.x = obstacle_dist.robot_position.odom_pose.x;
                obstacle_dist.robot_position.odom_pose_stack.y = obstacle_dist.robot_position.odom_pose.y;
                obstacle_dist.robot_position.odom_theta_stack = obstacle_dist.robot_position.odom_theta;
                if (mode == "CREATE")
                {
                    mapping(obstacle_dist.robot_position.robot_pose.position, obstacle_dist.robot_position.robot_theta, range_point_stack, angle_stack);
                }
                else if (mode == "NAVIGATION")
                {
                    data_publisher(obstacle_dist.robot_position.robot_pose.position, obstacle_dist.robot_position.robot_theta);
                }
                sleep(3);
            }
        }
        void localization(geometry_msgs::Point robot_pose, std::vector<geometry_msgs::Point> &range_point, std::vector<float> &angle, float diff_x, float diff_y, float diff_theta, float &true_diff_x, float &true_diff_y, float &true_diff_theta)
        {
            true_diff_x = diff_x;
            true_diff_y = diff_y;
            true_diff_theta = diff_theta;
            for (int i=0; i<angle.size(); i++)
            {
                angle[i] += true_diff_theta;
                while (ros::ok())
                {
                    if (std::fabs(angle[i]) > M_PI)
                    {
                        if (angle[i] > 0.)
                        {
                            angle[i] -= 2*M_PI;
                        }
                        else
                        {
                            angle[i] += 2*M_PI;
                        }
                    }
                    if (std::fabs(angle[i]) <= M_PI)
                    {
                        break;
                    }
                }
            }
            for (int i=0; i<range_point.size(); i++)
            {
                geometry_msgs::Point point;
                point.x = range_point[i].x;
                point.y = range_point[i].y;
                range_point[i].x = (point.x - robot_pose.x)*cos(true_diff_theta) - (point.y - robot_pose.y)*sin(true_diff_theta) + robot_pose.x + true_diff_x;
                range_point[i].y = (point.x - robot_pose.x)*sin(true_diff_theta) + (point.y - robot_pose.y)*cos(true_diff_theta) + robot_pose.y + true_diff_y;
            }
        }
        void mapping(geometry_msgs::Point robot_pose, float robot_theta, std::vector<geometry_msgs::Point> range_point, std::vector<float> angle)
        {
            robot_position.position.x = robot_pose.x;
            robot_position.position.y = robot_pose.y;
            robot_position.position.z = 0.0;

            robot_position.orientation.w = cos(robot_theta / 2.);
            robot_position.orientation.x = 0.00;
            robot_position.orientation.y = 0.00;
            robot_position.orientation.z = sin(robot_theta / 2.);
            // tf::Quaternion quat_msg = tf::createQuaternionFromRPY(0., 0., (prev_yaw + yaw));
            // quaternionTFToMsg(quat_msg, calculation_odom.pose.pose.orientation);

            geometry_msgs::Vector3 vec;
            vec.z = 0.0;
            for (int i=0; i<range_point.size(); i++)
            {
                ros::spinOnce();
                vector_2d[zero_point + gridding.float_to_int(gridding.float_to_grid(range_point[i].x))][zero_point + gridding.float_to_int(gridding.float_to_grid(range_point[i].y))] = 1;
                if ((zero_point + gridding.float_to_int(gridding.float_to_grid(range_point[i].x))) < limit_point[0])
                {
                    limit_point[0] = zero_point + gridding.float_to_int(gridding.float_to_grid(range_point[i].x));
                }
                if (limit_point[1] < (zero_point + gridding.float_to_int(gridding.float_to_grid(range_point[i].x))))
                {
                    limit_point[1] = zero_point + gridding.float_to_int(gridding.float_to_grid(range_point[i].x));
                }
                if ((zero_point + gridding.float_to_int(gridding.float_to_grid(range_point[i].y))) < limit_point[2])
                {
                    limit_point[2] = zero_point + gridding.float_to_int(gridding.float_to_grid(range_point[i].y));
                }
                if (limit_point[3] < (zero_point + gridding.float_to_int(gridding.float_to_grid(range_point[i].y))))
                {
                    limit_point[3] = zero_point + gridding.float_to_int(gridding.float_to_grid(range_point[i].y));
                }
                if ((((M_PI/4)) < std::fabs(angle[i])) && (std::fabs(angle[i]) < ((3*M_PI/4))))
                {
                    for (int j=1; j<((std::fabs(range_point[i].y - robot_position.position.y))/gridding.size); j++)
                    {
                        // if (vector_2d[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.x + ((j*gridding.size/tan(angle[i]))*((std::fabs(range_point[i].y-robot_position.position.y))/(range_point[i].y-robot_position.position.y)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.y + ((j*gridding.size                                     )*((std::fabs(range_point[i].y-robot_position.position.y))/(range_point[i].y-robot_position.position.y)))))] != 1)
                        // {
                        //     vector_2d[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.x + ((j*gridding.size/tan(angle[i]))*((std::fabs(range_point[i].y-robot_position.position.y))/(range_point[i].y-robot_position.position.y)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.y + ((j*gridding.size                                     )*((std::fabs(range_point[i].y-robot_position.position.y))/(range_point[i].y-robot_position.position.y)))))] = 0;
                        // }
                        vector_2d[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.x + ((j*gridding.size/tan(angle[i]))*((std::fabs(range_point[i].y-robot_position.position.y))/(range_point[i].y-robot_position.position.y)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.y + ((j*gridding.size                                     )*((std::fabs(range_point[i].y-robot_position.position.y))/(range_point[i].y-robot_position.position.y)))))] = 0;
                        if ((zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.x + ((j*gridding.size/tan(angle[i]))*((std::fabs(range_point[i].y-robot_position.position.y))/(range_point[i].y-robot_position.position.y)))))) < limit_point[0])
                        {
                            limit_point[0] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.x + ((j*gridding.size/tan(angle[i]))*((std::fabs(range_point[i].y-robot_position.position.y))/(range_point[i].y-robot_position.position.y)))));
                        }
                        if (limit_point[1] < (zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.x + ((j*gridding.size/tan(angle[i]))*((std::fabs(range_point[i].y-robot_position.position.y))/(range_point[i].y-robot_position.position.y)))))))
                        {
                            limit_point[1] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.x + ((j*gridding.size/tan(angle[i]))*((std::fabs(range_point[i].y-robot_position.position.y))/(range_point[i].y-robot_position.position.y)))));
                        }
                        if ((zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.y + ((j*gridding.size                                     )*((std::fabs(range_point[i].y-robot_position.position.y))/(range_point[i].y-robot_position.position.y)))))) < limit_point[2])
                        {
                            limit_point[2] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.y + ((j*gridding.size                                     )*((std::fabs(range_point[i].y-robot_position.position.y))/(range_point[i].y-robot_position.position.y)))));
                        }
                        if (limit_point[3] < (zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.y + ((j*gridding.size                                     )*((std::fabs(range_point[i].y-robot_position.position.y))/(range_point[i].y-robot_position.position.y)))))))
                        {
                            limit_point[3] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.y + ((j*gridding.size                                     )*((std::fabs(range_point[i].y-robot_position.position.y))/(range_point[i].y-robot_position.position.y)))));
                        }
                    }
                }
                else
                {
                    for (int j=1; j<((std::fabs(range_point[i].x - robot_position.position.x))/gridding.size); j++)
                    {
                        // if (vector_2d[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.x + ((j*gridding.size                                     )*((std::fabs(range_point[i].x-robot_position.position.x))/(range_point[i].x-robot_position.position.x)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.y + ((j*gridding.size*tan(angle[i]))*((std::fabs(range_point[i].x-robot_position.position.x))/(range_point[i].x-robot_position.position.x)))))] != 1)
                        // {
                        //     vector_2d[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.x + ((j*gridding.size                                     )*((std::fabs(range_point[i].x-robot_position.position.x))/(range_point[i].x-robot_position.position.x)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.y + ((j*gridding.size*tan(angle[i]))*((std::fabs(range_point[i].x-robot_position.position.x))/(range_point[i].x-robot_position.position.x)))))] = 0;
                        // }
                        vector_2d[zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.x + ((j*gridding.size                                     )*((std::fabs(range_point[i].x-robot_position.position.x))/(range_point[i].x-robot_position.position.x)))))][zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.y + ((j*gridding.size*tan(angle[i]))*((std::fabs(range_point[i].x-robot_position.position.x))/(range_point[i].x-robot_position.position.x)))))] = 0;
                        if ((zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.x + ((j*gridding.size                                     )*((std::fabs(range_point[i].x-robot_position.position.x))/(range_point[i].x-robot_position.position.x)))))) < limit_point[0])
                        {
                            limit_point[0] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.x + ((j*gridding.size                                     )*((std::fabs(range_point[i].x-robot_position.position.x))/(range_point[i].x-robot_position.position.x)))));
                        }
                        if (limit_point[1] < (zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.x + ((j*gridding.size                                     )*((std::fabs(range_point[i].x-robot_position.position.x))/(range_point[i].x-robot_position.position.x)))))))
                        {
                            limit_point[1] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.x + ((j*gridding.size                                     )*((std::fabs(range_point[i].x-robot_position.position.x))/(range_point[i].x-robot_position.position.x)))));
                        }
                        if ((zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.y + ((j*gridding.size*tan(angle[i]))*((std::fabs(range_point[i].x-robot_position.position.x))/(range_point[i].x-robot_position.position.x)))))) < limit_point[2])
                        {
                            limit_point[2] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.y + ((j*gridding.size*tan(angle[i]))*((std::fabs(range_point[i].x-robot_position.position.x))/(range_point[i].x-robot_position.position.x)))));
                        }
                        if (limit_point[3] < (zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.y + ((j*gridding.size*tan(angle[i]))*((std::fabs(range_point[i].x-robot_position.position.x))/(range_point[i].x-robot_position.position.x)))))))
                        {
                            limit_point[3] = zero_point + gridding.float_to_int(gridding.float_to_grid(robot_position.position.y + ((j*gridding.size*tan(angle[i]))*((std::fabs(range_point[i].x-robot_position.position.x))/(range_point[i].x-robot_position.position.x)))));
                        }
                    }
                }
            }
            for (int i=0; i<range_point.size(); i++)
            {
                vector_2d[zero_point + gridding.float_to_int(gridding.float_to_grid(range_point[i].x))][zero_point + gridding.float_to_int(gridding.float_to_grid(range_point[i].y))] = 1;
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
            pub_robot_position.publish(robot_position);
            ros::spinOnce();
            map.cost.clear();
            map.clearly.clear();
        }
        void data_publisher(geometry_msgs::Point robot_pose, float robot_theta)
        {
            ros::spinOnce();

            robot_position.position.x = robot_pose.x;
            robot_position.position.y = robot_pose.y;
            robot_position.position.z = 0.0;

            robot_position.orientation.w = cos(robot_theta / 2.);
            robot_position.orientation.x = 0.00;
            robot_position.orientation.y = 0.00;
            robot_position.orientation.z = sin(robot_theta / 2.);

            pub_map.publish(map);
            pub_robot_position.publish(robot_position);
            ros::spinOnce();
        }
        void get_map()
        {
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
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_slam");
    SLAM slam;
    ros::spin();
    return 0;
}