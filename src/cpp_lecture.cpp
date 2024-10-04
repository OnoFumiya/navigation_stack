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
#include <visualization_msgs/MarkerArray.h>
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



class RvizMarkerLibrary
{
    public:
        RvizMarkerLibrary()
        {}
        visualization_msgs::Marker makeMarker(const int type, const std::string& frame_id, const std::string& name, const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& scale, const std_msgs::ColorRGBA& color, const ros::Duration& lifetime)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = name;
            marker.id = 0;
            marker.type = type;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale = scale;
            marker.color = color;
            marker.lifetime = lifetime;
            marker.pose = pose;
            return marker;
        }
        visualization_msgs::Marker makeMarkerList(const int type, const std::string& frame_id, const std::string& name, const geometry_msgs::Pose& pose, const std::vector<geometry_msgs::Point>& points, const geometry_msgs::Vector3& scale, const std::vector<std_msgs::ColorRGBA>& colors, const ros::Duration& lifetime)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = name;
            marker.id = 0;
            marker.type = type;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale = scale;
            marker.points = points;
            marker.colors = colors;
            marker.lifetime = lifetime;
            marker.pose = pose;
            return marker;
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_lecture");
    std::vector<double> func_2d_x, func_2d_y;
    std::vector<double> func_sig_x, func_sig_y;
    std::vector<double> x({8,-3,-8});
    std::vector<double> y({-7,-5,0});

    // 放物線
    double a, b, c;
    a = (x[0]*(y[2] - y[1]) + x[1]*(y[0] - y[2]) + x[2]*(y[1] - y[0])) / ((x[0] - x[1]) * (x[0] - x[2]) * (x[1] - x[2]));
    b = ((std::pow(x[1], 2) - std::pow(x[2], 2))*y[0] + (std::pow(x[2], 2) - std::pow(x[0], 2))*y[1] + (std::pow(x[0], 2) - std::pow(x[1], 2))*y[2]) / ((x[0] - x[1]) * (x[0] - x[2]) * (x[2] - x[1]));
    c = y[0] - a*std::pow(x[0], 2) -b*x[0];

    // シグモイド曲線
    double L, a_sig, shift_x, shift_y;
    shift_x = (x[0] - x[2]) / 2 - x[0];
    if (y[0] < y[2]) {
        L = y[2] - y[0];
        shift_y = y[0];
    }
    else {
        L = y[0] - y[2];
        shift_y = y[2];
    }
    a_sig = (y[1] - L/2) / (x[1] - 0);

    // 描画
    for (int i=-1000; i<1000; i++) {
        double ix = ((double)i/100);
        func_2d_x.push_back(ix);
        func_2d_y.push_back(a*std::pow(ix, 2) + b*ix + c);

        func_sig_x.push_back(ix);
        // func_sig_y.push_back(L / (1.0 + exp(-k * ((ix) - x_c))));
        func_sig_y.push_back(L / (1 + exp((-4.0 * b * (ix + shift_x))/L)) + shift_y);
    }
    printf("%f,  %f\n",shift_x,shift_y);
    printf("%f, %f %f\na = %f\n", (100 / (1 + exp(-4 * -0.01))), (100 / (1 + exp(-4 * 0))), (100 / (1 + exp(-4 * 0.0001))), ((100 / (1 + exp(-4 * 0.0001))) - (100 / (1 + exp(-4 * -0.0001)))) / 0.0002);

    matplotlibcpp::plot(func_2d_x,func_2d_y, "--y");
    matplotlibcpp::plot(func_sig_x,func_sig_y, "--b");
    matplotlibcpp::plot(x, y, "or");
    matplotlibcpp::show();

}