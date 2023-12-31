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
// using namespace Eigen;
// using namespace std;


class TEST
{
    private:
        ros::Subscriber sub_scan;
        bool start = false;
        void callback_scan(const sensor_msgs::LaserScan &msg)
        {
            scan_point.clear();
            geometry_msgs::Point pt;
            pt.z = 0.2;
            for (int i=0; i<msg.ranges.size(); i++)
            {
                if (std::isnan(msg.ranges[i]) == false)
                {
                    pt.x = msg.ranges[i]*(cos(msg.angle_min + msg.angle_increment*i));
                    pt.y = msg.ranges[i]*(sin(msg.angle_min + msg.angle_increment*i));
                    scan_point.push_back(pt);
                }
            }
            start = true;
        }
    public:
        std::vector<geometry_msgs::Point> scan_point;
        TEST()
        {
            ros::NodeHandle nh;
            sub_scan = nh.subscribe("/scan", 10, &TEST::callback_scan, this);
            while (ros::ok())
            {
                ros::spinOnce();
                if (start)
                {
                    break;
                }
            }
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_lecture");
    TEST test;
    std::vector<float> x_pt;
    std::vector<float> y_pt;
    while (ros::ok())
    {
        ros::spinOnce();
        matplotlibcpp::cla();
        matplotlibcpp::xlim(-1,4);
        matplotlibcpp::ylim(-2,2);
        for (int i=0; i<test.scan_point.size(); i++)
        {
            matplotlibcpp::plot({test.scan_point[i].x},{test.scan_point[i].y},".g");
        }
        matplotlibcpp::pause(0.01);
        ros::spinOnce();
        ROS_INFO("PLAY");
    }
    // ros::NodeHandle nh;
    // for (int i=0; i<10; i++)
    // {
    //     matplotlibcpp::cla();
    //     matplotlibcpp::xlim(0,6);
    //     matplotlibcpp::ylim(0,101);
    //     matplotlibcpp::plot({1,5},{(float)(i+1),(float)((i+1)*(i+1))},"og");
    //     matplotlibcpp::pause(1);
    // }
    // ROS_INFO("end");
    // ros::NodeHandle pnh;
    // XmlRpc::XmlRpcValue param;
    // pnh.getParam("lidar_pose", param);
    // // cout << param[0] << endl;
    // pnh.getParam("dr_spaam_topic", param);
    // // cout << param << endl;
    // std::string id;
    // if (param.getType() == XmlRpc::XmlRpcValue::TypeString)
    // {
    //     id = static_cast<std::string>(param);
    //     ROS_INFO("True");
    // }
    // ROS_INFO("%s",id.c_str());
    // // ros::load
    // // std::vector<float> x_points = {0  , 1  , 2  , 3  };
    // // std::vector<float> y_points = {0.1, 2.1, 4.1, 6.1};

    // // // Initialize matrix for least squares
    // // MatrixXd A(x_points.size(), 2);
    // // VectorXd b(x_points.size());

    // // // Fill the matrix A and vector b
    // // for (int i = 0; i < x_points.size(); i++) {
    // //     A(i, 0) = x_points[i];
    // //     A(i, 1) = 1.0;
    // //     b(i) = y_points[i];
    // // }

    // // // Solve the least squares problem
    // // Vector2d result = A.colPivHouseholderQr().solve(b);

    // // // Result contains the slope (result[0]) and the intercept (result[1])
    // // std::cout << "Slope: " << result[0] << ", Intercept: " << result[1] << "\n";
    
    ros::spinOnce();
    ros::spin();
    return 0;
}