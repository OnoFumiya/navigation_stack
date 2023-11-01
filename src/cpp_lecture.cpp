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


// class TEST
// {
//     public:
//         TEST()
//         {}
// };


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_lecture");
    ros::NodeHandle pnh;
    XmlRpc::XmlRpcValue param;
    pnh.getParam("lidar_pose", param);
    // cout << param[0] << endl;
    pnh.getParam("dr_spaam_topic", param);
    // cout << param << endl;
    std::string id;
    if (param.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
        id = static_cast<std::string>(param);
        ROS_INFO("True");
    }
    ROS_INFO("%s",id.c_str());
    // ros::load
    // std::vector<float> x_points = {0  , 1  , 2  , 3  };
    // std::vector<float> y_points = {0.1, 2.1, 4.1, 6.1};

    // // Initialize matrix for least squares
    // MatrixXd A(x_points.size(), 2);
    // VectorXd b(x_points.size());

    // // Fill the matrix A and vector b
    // for (int i = 0; i < x_points.size(); i++) {
    //     A(i, 0) = x_points[i];
    //     A(i, 1) = 1.0;
    //     b(i) = y_points[i];
    // }

    // // Solve the least squares problem
    // Vector2d result = A.colPivHouseholderQr().solve(b);

    // // Result contains the slope (result[0]) and the intercept (result[1])
    // std::cout << "Slope: " << result[0] << ", Intercept: " << result[1] << "\n";
    
    ros::spinOnce();
    ros::spin();
    return 0;
}