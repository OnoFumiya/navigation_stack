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
            scan_points.clear();
            g_point.clear();
            geometry_msgs::Point pt;
            pt.z = 0.2;
            for (int i=0; i<msg.ranges.size(); i++)
            {
                if ((std::isnan(msg.ranges[i]) == false) && (msg.ranges[i] > 0.1))
                {
                    pt.x = msg.ranges[i]*(cos(msg.angle_min + msg.angle_increment*i));
                    pt.y = msg.ranges[i]*(sin(msg.angle_min + msg.angle_increment*i));
                    bool inputer = false;
                    for (int j=0; j<scan_points.size(); j++)
                    {
                        if (sqrtf(powf(scan_points[j][scan_points[j].size()-1].x - pt.x, 2) + powf(scan_points[j][scan_points[j].size()-1].y - pt.y, 2)) < 0.1)
                        {
                            g_point[j].x = (g_point[j].x*scan_points[j].size() + pt.x) / (scan_points[j].size()+1);
                            g_point[j].y = (g_point[j].y*scan_points[j].size() + pt.y) / (scan_points[j].size()+1);
                            scan_points[j].push_back(pt);
                            inputer = true;
                            break;
                        }
                    }
                    if (inputer != true)
                    {
                        std::vector<geometry_msgs::Point> new_scan_point;
                        new_scan_point.push_back(pt);
                        scan_points.push_back(new_scan_point);
                        g_point.push_back(pt);
                    }
                }
            }
            for (int i=scan_points.size()-1; 0<=i; i--)
            {
                if (0.5 < sqrtf(powf(scan_points[i][scan_points[i].size()-1].x - scan_points[i][0].x, 2) + powf(scan_points[i][scan_points[i].size()-1].y - scan_points[i][0].y, 2)))
                {
                    g_point.erase(g_point.begin() + i);
                }
            }
            start = true;
        }
    public:
        std::vector<std::vector<geometry_msgs::Point>> scan_points;
        std::vector<geometry_msgs::Point> g_point;
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
        for (int i=0; i<test.scan_points.size(); i++)
        {
            for (int j=0; j<test.scan_points[i].size(); j++)
            {
                if (i==0)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".r");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xr");
                }
                else if (i==1)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".g");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xg");
                }
                else if (i==2)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".b");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xb");
                }
                else if (i==3)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".y");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xy");
                }
                else if (i==4)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".c");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xc");
                }
                else if (i==5)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".m");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xm");
                }
                else if (i==6)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".r");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xr");
                }
                else if (i==7)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".g");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xg");
                }
                else if (i==8)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".b");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xb");
                }
                else if (i==9)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".y");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xy");
                }
                else if (i==10)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".c");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xc");
                }
                else if (i==11)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".m");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xm");
                }
                else if (i==12)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".r");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xr");
                }
                else if (i==13)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".g");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xg");
                }
                else if (i==14)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".b");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xb");
                }
                else if (i==15)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".y");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xy");
                }
                else if (i==16)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".c");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xc");
                }
                else if (i==17)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".m");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xm");
                }
                else if (i==18)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".r");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xr");
                }
                else if (i==19)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".g");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xg");
                }
                else if (i==20)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".b");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xb");
                }
                else if (i==21)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".y");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xy");
                }
                else if (i==22)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".c");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xc");
                }
                else if (i==23)
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".m");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xm");
                }
                else
                {
                    matplotlibcpp::plot({test.scan_points[i][j].x},{test.scan_points[i][j].y},".k");
                    matplotlibcpp::plot({test.g_point[i].x},{test.g_point[i].y},"xk");
                }
            }
        }
        matplotlibcpp::pause(0.1);
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