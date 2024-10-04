#include <stdio.h>
#include <ros/ros.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
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


class Trajectory
{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub_marker_ob;
        ros::Publisher pub_marker_type1;
        ros::Publisher pub_marker_type2;
        std::vector<std_msgs::ColorRGBA> ob_colors;
        std::vector<std_msgs::ColorRGBA> type1_colors;
        std::vector<std_msgs::ColorRGBA> type2_colors;
        ros::Subscriber point_sub;
        RvizMarkerLibrary marker_lib;
        void callback_point(const geometry_msgs::PointStamped &point) {
            if (mode == 0) {
                x.push_back(point.point.x);
                y.push_back(point.point.y);
                if (x.size() >= 3) {
                    mode = 1;
                    parabola();
                    sigmoid();
                }
            }
            else {
                x[mode-1] = point.point.x;
                y[mode-1] = point.point.y;
                mode += 1;
                if (mode >= 4) {
                    mode = 1;
                    parabola();
                    sigmoid();
                }
            }
            update();
        }
    public:
        std::vector<geometry_msgs::Point> ob_point;
        std::vector<geometry_msgs::Point> type1_pose;
        std::vector<geometry_msgs::Point> type2_pose;
        std::vector<double> func_2d_x, func_2d_y;
        std::vector<double> func_sig_x, func_sig_y;
        // std::vector<double> x({8,-3,-8});
        // std::vector<double> y({-7,-5,0});
        std::vector<double> x;
        std::vector<double> y;
        geometry_msgs::Pose base_pose;
        geometry_msgs::Vector3 line_size, ob_size;
        std_msgs::ColorRGBA black, white, red, blue, cyan, yellow, green, purple, orange;
        int mode;
        double a, b, c;
        double L, a_sig, shift_x, shift_y;
        Trajectory() {
            base_pose.position.x = 0.0;
            base_pose.position.y = 0.0;
            base_pose.position.z = 0.1;
            base_pose.orientation.w = 1.0;
            base_pose.orientation.x = 0.0;
            base_pose.orientation.y = 0.0;
            base_pose.orientation.z = 0.0;
            line_size.x = 0.02;
            line_size.y = 0.02;
            line_size.z = 0.02;
            ob_size.x = 0.05;
            ob_size.y = 0.05;
            ob_size.z = 0.02;
            black.a = 1.;
            black.r = 0.;
            black.g = 0.;
            black.b = 0.;
            white.a = 1.;
            white.r = 1.;
            white.g = 1.;
            white.b = 1.;
            red.a = 1.;
            red.r = 1.;
            red.g = 0.;
            red.b = 0.;
            blue.a = 1.;
            blue.r = 0.;
            blue.g = 0.;
            blue.b = 1.;
            cyan.a = 1.;
            cyan.r = 0.;
            cyan.g = 1.;
            cyan.b = 1.;
            yellow.a = 1.;
            yellow.r = 1.;
            yellow.g = 1.;
            yellow.b = 0.;
            green.a = 1.;
            green.r = 0.;
            green.g = 1.;
            green.b = 0.;
            purple.a = 1.;
            purple.r = 0.5;
            purple.g = 0.;
            purple.b = 0.5;
            orange.a = 1.;
            orange.r = 1.;
            orange.g = 0.66;
            orange.b = 0.;
            mode = 0;
            pub_marker_ob = nh.advertise<visualization_msgs::Marker>("/predict/ob", 1);
            pub_marker_type1 = nh.advertise<visualization_msgs::Marker>("/predict/type1", 1);
            pub_marker_type2 = nh.advertise<visualization_msgs::Marker>("/predict/type2", 1);
            point_sub = nh.subscribe("/clicked_point", 1, &Trajectory::callback_point, this);
            viewer();
        }
        // 放物線
        void parabola() {
            a = (x[0]*(y[2] - y[1]) + x[1]*(y[0] - y[2]) + x[2]*(y[1] - y[0])) / ((x[0] - x[1]) * (x[0] - x[2]) * (x[1] - x[2]));
            b = ((std::pow(x[1], 2) - std::pow(x[2], 2))*y[0] + (std::pow(x[2], 2) - std::pow(x[0], 2))*y[1] + (std::pow(x[0], 2) - std::pow(x[1], 2))*y[2]) / ((x[0] - x[1]) * (x[0] - x[2]) * (x[2] - x[1]));
            c = y[0] - a*std::pow(x[0], 2) -b*x[0];
        }
        // シグモイド曲線
        void sigmoid() {
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
        }
        // 更新
        void update() {
            func_2d_x.clear();
            func_2d_y.clear();
            func_sig_x.clear();
            func_sig_y.clear();
            if (x.size() == 3) {
                for (int i=-1000; i<1000; i++) {
                    double ix = ((double)i/100);
                    // 放物線
                    func_2d_x.push_back(ix);
                    func_2d_y.push_back(a*std::pow(ix, 2) + b*ix + c);

                    // シグモイド曲線
                    func_sig_x.push_back(ix);
                    func_sig_y.push_back(L / (1 + exp((-4.0 * b * (ix + shift_x))/L)) + shift_y);
                }
            }

            // ROS化
            geometry_msgs::Point pt;
            ob_point.clear();
            for (int i=0; i<x.size(); i++) {
                pt.x = x[i];
                pt.y = y[i];
                pt.z = 0.02;
                ob_point.push_back(pt);
            }
            type1_pose.clear();
            type2_pose.clear();
            for (int i=0; i<func_2d_x.size(); i++) {
                pt.x = func_2d_x[i];
                pt.y = func_2d_y[i];
                pt.z = 0.01;
                type1_pose.push_back(pt);
                pt.x = func_sig_x[i];
                pt.y = func_sig_y[i];
                pt.z = 0.01;
                type2_pose.push_back(pt);
            }
            ob_colors.resize(ob_point.size(), red);
            type1_colors.resize(type1_pose.size(), blue);
            type2_colors.resize(type2_pose.size(), yellow);
        }
        // 描画
        void viewer() {
            ros::spinOnce();
            while (ros::ok()) {
                if (x.size() != 0) pub_marker_ob.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::POINTS, "map", "obstacle", base_pose, ob_point, ob_size, ob_colors, ros::Duration(1) ) );
                pub_marker_type1.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::LINE_STRIP, "map", "type1", base_pose, type1_pose, line_size, type1_colors, ros::Duration(1) ) );
                pub_marker_type2.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::LINE_STRIP, "map", "type2", base_pose, type2_pose, line_size, type2_colors, ros::Duration(1) ) );
                ros::spinOnce();
            }
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_lecture");
    Trajectory traj;
    // std::vector<double> func_2d_x, func_2d_y;
    // std::vector<double> func_sig_x, func_sig_y;
    // std::vector<double> x({8,-3,-8});
    // std::vector<double> y({-7,-5,0});

    // // 放物線
    // double a, b, c;
    // a = (x[0]*(y[2] - y[1]) + x[1]*(y[0] - y[2]) + x[2]*(y[1] - y[0])) / ((x[0] - x[1]) * (x[0] - x[2]) * (x[1] - x[2]));
    // b = ((std::pow(x[1], 2) - std::pow(x[2], 2))*y[0] + (std::pow(x[2], 2) - std::pow(x[0], 2))*y[1] + (std::pow(x[0], 2) - std::pow(x[1], 2))*y[2]) / ((x[0] - x[1]) * (x[0] - x[2]) * (x[2] - x[1]));
    // c = y[0] - a*std::pow(x[0], 2) -b*x[0];

    // // シグモイド曲線
    // double L, a_sig, shift_x, shift_y;
    // shift_x = (x[0] - x[2]) / 2 - x[0];
    // if (y[0] < y[2]) {
    //     L = y[2] - y[0];
    //     shift_y = y[0];
    // }
    // else {
    //     L = y[0] - y[2];
    //     shift_y = y[2];
    // }
    // a_sig = (y[1] - L/2) / (x[1] - 0);

    // // 描画
    // for (int i=-1000; i<1000; i++) {
    //     double ix = ((double)i/100);
    //     func_2d_x.push_back(ix);
    //     func_2d_y.push_back(a*std::pow(ix, 2) + b*ix + c);

    //     func_sig_x.push_back(ix);
    //     func_sig_y.push_back(L / (1 + exp((-4.0 * b * (ix + shift_x))/L)) + shift_y);
    // }
    // printf("%f,  %f\n",shift_x,shift_y);
    // printf("%f, %f %f\na = %f\n", (100 / (1 + exp(-4 * -0.01))), (100 / (1 + exp(-4 * 0))), (100 / (1 + exp(-4 * 0.0001))), ((100 / (1 + exp(-4 * 0.0001))) - (100 / (1 + exp(-4 * -0.0001)))) / 0.0002);

    // matplotlibcpp::plot(func_2d_x,func_2d_y, "--y");
    // matplotlibcpp::plot(func_sig_x,func_sig_y, "--b");
    // matplotlibcpp::plot(x, y, "ob");
    // matplotlibcpp::show();

}