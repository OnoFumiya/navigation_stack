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
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
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
#include <nlopt.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
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

// std::vector<double> x;
// std::vector<double> y;
// double equation(const std::vector<double> &sx, std::vector<double> &grad, void *my_func_data) {
//     double term1 = (y[0] - y[1]) / (1. / (1. + exp(-sx[0] * (x[0] - x[1]))) - 0.5);
//     double term2 = (y[2] - y[1]) / (1. / (1. + exp(-sx[0] * (x[2] - x[1]))) - 0.5);
//     return term1 - term2; // 方程式の値を返す
// }

class Trajectory
{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub_image;
        ros::Publisher pub_marker_ob;
        ros::Publisher pub_marker_type1;
        ros::Publisher pub_marker_type2;
        ros::Publisher pub_marker_type3;
        ros::Publisher pub_marker_type4;
        ros::Publisher pub_marker_type5;
        std::vector<std_msgs::ColorRGBA> ob_colors;
        std::vector<std_msgs::ColorRGBA> type1_colors;
        std::vector<std_msgs::ColorRGBA> type2_colors;
        std::vector<std_msgs::ColorRGBA> type3_colors;
        std::vector<std_msgs::ColorRGBA> type4_colors;
        std::vector<std_msgs::ColorRGBA> type5_colors;
        ros::Subscriber point_sub;
        RvizMarkerLibrary marker_lib;
        bool set_pt, no_path_del;
        void callback_point(const geometry_msgs::PointStamped &point) {
            if (mode == 0) {
                x.push_back(point.point.x);
                y.push_back(point.point.y);
                if (x.size() >= 3) {
                    mode = 1;
                    line_path();
                    parabola();
                    sigmoid();
                }
            }
            else {
                if (set_pt) {
                    x[mode-1] = point.point.x;
                    y[mode-1] = point.point.y;
                    mode += 1;
                    if (mode >= 4) {
                        mode = 1;
                        line_path();
                        parabola();
                        sigmoid();
                    }
                } else {
                    x[0] = x[1];
                    y[0] = y[1];
                    x[1] = x[2];
                    y[1] = y[2];
                    x[2] = point.point.x;
                    y[2] = point.point.y;
                    line_path();
                    parabola();
                    sigmoid();
                }
            }
            update();
        }
    public:
        nav_msgs::OccupancyGrid base_map;
        std::vector<geometry_msgs::Point> ob_point;
        std::vector<geometry_msgs::Point> type1_pose;
        std::vector<geometry_msgs::Point> type2_pose;
        std::vector<geometry_msgs::Point> type3_pose;
        std::vector<geometry_msgs::Point> type4_pose;
        std::vector<geometry_msgs::Point> type5_pose;
        bool type1_check, type2_check, type3_check, type4_check, type5_check;
        std::vector<double> func_1d_x,   func_1d_y;
        std::vector<double> func_2d_x,   func_2d_y;
        std::vector<double> func_t2d_x,  func_t2d_y;
        std::vector<double> func_sig_x,  func_sig_y;
        std::vector<double> func_tsig_x, func_tsig_y;
        std::vector<double> x;
        std::vector<double> y;
        geometry_msgs::Pose base_pose;
        geometry_msgs::Vector3 line_size, ob_size;
        std_msgs::ColorRGBA black, white, red, blue, cyan, yellow, green, purple, orange, pink;
        int mode;
        double la, lb;
        double a, b, c;
        double ta, tb, tc;
        double L, a_sig, shift_x, shift_y;
        double tL, ta_sig, tshift_x, tshift_y;
        Trajectory() {
            base_pose.position.x = 0.0;
            base_pose.position.y = 0.0;
            base_pose.position.z = 0.0;
            base_pose.orientation.w = 1.0;
            base_pose.orientation.x = 0.0;
            base_pose.orientation.y = 0.0;
            base_pose.orientation.z = 0.0;
            line_size.x = 0.1;
            line_size.y = 0.1;
            line_size.z = 0.1;
            ob_size.x = 0.15;
            ob_size.y = 0.15;
            ob_size.z = 0.15;
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
            orange.g = 0.55;
            orange.b = 0.;
            pink.a = 1.;
            pink.r = 1.;
            pink.g = 0.53;
            pink.b = 0.74;
            mode = 0;
            pub_image = nh.advertise<nav_msgs::OccupancyGrid>("/predict/base", 1);
            pub_marker_ob = nh.advertise<visualization_msgs::Marker>("/predict/ob", 1);
            pub_marker_type1 = nh.advertise<visualization_msgs::Marker>("/predict/type1", 1);
            pub_marker_type2 = nh.advertise<visualization_msgs::Marker>("/predict/type2", 1);
            pub_marker_type3 = nh.advertise<visualization_msgs::Marker>("/predict/type3", 1);
            pub_marker_type4 = nh.advertise<visualization_msgs::Marker>("/predict/type4", 1);
            pub_marker_type5 = nh.advertise<visualization_msgs::Marker>("/predict/type5", 1);
            set_pt = nh.param<bool>( "/predict_traj/set_pt", true );
            no_path_del = nh.param<bool>( "/predict_traj/no_path_del", true );
            point_sub = nh.subscribe("/clicked_point", 1, &Trajectory::callback_point, this);
            set_base_map(nh.param<std::string>( "/predict_traj/base_image_path", "/home/sobits/catkin_ws/src/navigation_stack/image/XY.jpg" ));
            viewer();
        }
        // 近似直線
        void line_path() {
            // 例外
            type1_check = true;
            if ((3*(std::pow(x[0], 2) + std::pow(x[1], 2) + std::pow(x[2], 2))) == std::pow(x[0] + x[1] + x[2], 2)) type1_check = false;

            // 算出
            if (type1_check) {
                double sum_x, sum_y, sum_xx, sum_xy;
                sum_x = x[0] + x[1] + x[2];
                sum_y = y[0] + y[1] + y[2];
                sum_xx = std::pow(x[0], 2) + std::pow(x[1], 2) + std::pow(x[2], 2);
                sum_xy = x[0]*y[0] + x[1]*y[1] + x[2]*y[2];
                la = (3.0 * sum_xy - sum_x * sum_y) / (3.0 * sum_xx - sum_x * sum_x);
                lb = (sum_y - la * sum_x) / 3.0;
            }

            // 棄却許容判定
            if (type1_check) {
                double samp_x1, samp_x2, samp_x3, samp_y1, samp_y2, samp_y3;
                samp_x1 = (-la*lb + la*y[0] + x[0]) / (std::pow(la, 2.) + 1.);
                samp_y1 = (-la*lb + la*y[0] + x[0]) / (std::pow(la, 2.) + 1.) + lb;
                samp_x2 = (-la*lb + la*y[1] + x[1]) / (std::pow(la, 2.) + 1.);
                samp_y2 = (-la*lb + la*y[1] + x[1]) / (std::pow(la, 2.) + 1.) + lb;
                samp_x3 = (-la*lb + la*y[2] + x[2]) / (std::pow(la, 2.) + 1.);
                samp_y3 = (-la*lb + la*y[2] + x[2]) / (std::pow(la, 2.) + 1.) + lb;
                if (((((samp_x1 <= samp_x2) && (samp_x2 <= samp_x3)) || ((samp_x3 <= samp_x2) && (samp_x2 <= samp_x1))) && (samp_x3 != samp_x1)) ||
                    ((((samp_y1 <= samp_y2) && (samp_y2 <= samp_y3)) || ((samp_y3 <= samp_y2) && (samp_y2 <= samp_y1))) && (samp_y3 != samp_y1))) {
                        type1_check = true;
                } else  type1_check = false;
            }
        }
        // 放物線
        void parabola() {
            // 例外
            type2_check = true;
            type3_check = true;
            if ((x[0] == x[1]) || (x[1] == x[2]) || (x[2] == x[0])) type2_check = false;
            if ((y[0] == y[1]) || (y[1] == y[2]) || (y[2] == y[0])) type3_check = false;
            // if ((std::fabs(x[0]-x[1]) < 0.001) || (std::fabs(x[1]-x[2]) < 0.001) || (std::fabs(x[2]-x[0]) < 0.001)) type2_check = false;
            // if ((std::fabs(y[0]-y[1]) < 0.001) || (std::fabs(y[1]-y[2]) < 0.001) || (std::fabs(y[2]-y[0]) < 0.001)) type3_check = false;

            // 算出
            if (type2_check) {
                a = (x[0]*(y[2] - y[1]) + x[1]*(y[0] - y[2]) + x[2]*(y[1] - y[0])) / ((x[0] - x[1]) * (x[0] - x[2]) * (x[1] - x[2]));
                b = ((std::pow(x[1], 2) - std::pow(x[2], 2))*y[0] + (std::pow(x[2], 2) - std::pow(x[0], 2))*y[1] + (std::pow(x[0], 2) - std::pow(x[1], 2))*y[2]) / ((x[0] - x[1]) * (x[0] - x[2]) * (x[2] - x[1]));
                c = y[0] - a*std::pow(x[0], 2) -b*x[0];
            }
            if (type3_check) {
                ta= (y[0]*(x[2] - x[1]) + y[1]*(x[0] - x[2]) + y[2]*(x[1] - x[0])) / ((y[0] - y[1]) * (y[0] - y[2]) * (y[1] - y[2]));
                tb= ((std::pow(y[1], 2) - std::pow(y[2], 2))*x[0] + (std::pow(y[2], 2) - std::pow(y[0], 2))*x[1] + (std::pow(y[0], 2) - std::pow(y[1], 2))*x[2]) / ((y[0] - y[1]) * (y[0] - y[2]) * (y[2] - y[1]));
                tc= x[0] - ta*std::pow(y[0], 2) -tb*y[0];
            }

            // 棄却許容判定
            if (type2_check) {
                if ((((x[0] <= x[1]) && (x[1] <= x[2])) || ((x[2] <= x[1]) && (x[1] <= x[0]))) && (x[2] != x[0])) {
                        type2_check = true;
                } else  type2_check = false;
                double pre_x, pre_y;
                pre_x = ( ((std::pow(2*a*x[2]+b,2) + 1)*x[2]) + ((x[2]-x[1])/std::fabs(x[2]-x[1])) * std::sqrt(std::pow((std::pow(2*a*x[2]+b,2) + 1)*x[2],2) - (std::pow(2*a*x[2]+b,2) + 1)*(std::pow(x[2],2)*(std::pow(2*a*x[2]+b,2) + 1)-1)) ) / ( std::pow(2*a*x[2]+b,2) + 1 );
                pre_y = (2*a*x[2] + b)*pre_x + y[2] - (2*a*x[2] + b) * x[2];
                if ((((x[0] < x[2]) && (x[2] < -(b)/(2*a))) || ((-(b)/(2*a) < x[2]) && (x[2] < x[0]))) || 
                    ( acos(   ( (x[2]-x[1]) * (pre_x-x[2]) + (y[2]-y[1]) * (pre_y-y[2]) ) / std::sqrt(std::pow(x[2]-x[1], 2) + std::pow(y[2]-y[1], 2))   ) > M_PI/2. )) {
                    type2_check = false;
                }
            }
            if (type3_check) {
                if ((((y[0] <= y[1]) && (y[1] <= y[2])) || ((y[2] <= y[1]) && (y[1] <= y[0]))) && (y[2] != y[0])) {
                        type3_check = true;
                } else  type3_check = false;
                double pre_x, pre_y;
                pre_y = ( ((std::pow(2*ta*y[2]+tb,2) + 1)*y[2]) + ((y[2]-y[1])/std::fabs(y[2]-y[1])) * std::sqrt(std::pow((std::pow(2*ta*y[2]+tb,2) + 1)*y[2],2) - (std::pow(2*ta*y[2]+tb,2) + 1)*(std::pow(y[2],2)*(std::pow(2*ta*y[2]+tb,2) + 1)-1)) ) / ( std::pow(2*ta*y[2]+tb,2) + 1 );
                pre_x = (2*ta*y[2] + tb)*pre_y + x[2] - (2*ta*y[2] + tb) * y[2];
                if ((((y[0] < y[2]) && (y[2] < -(tb)/(2*ta))) || ((-(tb)/(2*ta) < y[2]) && (y[2] < y[0]))) || 
                    ( acos(   ( (y[2]-y[1]) * (pre_y-y[2]) + (x[2]-x[1]) * (pre_x-x[2]) ) / std::sqrt(std::pow(y[2]-y[1], 2) + std::pow(x[2]-x[1], 2))   ) > M_PI/2. )) {
                    type3_check = false;
                }
            }
        }
        // シグモイド曲線
        void sigmoid() {
            // 例外
            type4_check = true;
            type5_check = true;
            if ((y[2] == y[1]) || (x[2] == x[1])) {
                type4_check = false;
                type5_check = false;
            }

            // 算出
            if (type4_check && type5_check) {
                shift_x = x[1];
                double theta;
                L = 2 * std::fabs(y[2]-y[1]);
                theta = (y[2] - y[1]) / (x[2] - x[1]);
                shift_y = y[1] - (L / 2.0);
                a_sig = 4 * theta / L;

                tshift_y = y[1];
                double ttheta;
                tL = 2 * std::fabs(x[2]-x[1]);
                ttheta = (x[2] - x[1]) / (y[2] - y[1]);
                tshift_x = x[1] - (tL / 2.0);
                ta_sig = 4 * ttheta / tL;
            }

            // 棄却許容判定
            if (type4_check && type5_check) {
                if ((((x[0] <= x[1]) && (x[1] <= x[2])) || ((x[2] <= x[1]) && (x[1] <= x[0]))) && (x[2] != x[0])) {
                        type4_check = true;
                } else  type4_check = false;
                if ((((y[0] <= y[1]) && (y[1] <= y[2])) || ((y[2] <= y[1]) && (y[1] <= y[0]))) && (y[2] != y[0])) {
                        type5_check = true;
                } else  type5_check = false;
            }
        }
        // 更新
        void update() {
            func_1d_x.clear();
            func_1d_y.clear();
            func_2d_x.clear();
            func_2d_y.clear();
            func_t2d_x.clear();
            func_t2d_y.clear();
            func_sig_x.clear();
            func_sig_y.clear();
            func_tsig_x.clear();
            func_tsig_y.clear();
            if (x.size() == 3) {
                for (int i=-3000; i<3000; i++) {
                    double ix = ((double)i/100);
                    double iy = ((double)i/100);
                    // 直線
                    func_1d_x.push_back(ix);
                    func_1d_y.push_back(la*ix + lb);

                    // 放物線
                    func_2d_x.push_back(ix);
                    func_2d_y.push_back(a*std::pow(ix, 2) + b*ix + c);
                    func_t2d_y.push_back(iy);
                    func_t2d_x.push_back(ta*std::pow(iy, 2) + tb*iy + tc);

                    // シグモイド曲線
                    func_sig_x.push_back(ix);
                    func_sig_y.push_back(L / (1.0 + exp(-a_sig * (ix - shift_x))) + shift_y);
                    func_tsig_y.push_back(iy);
                    func_tsig_x.push_back(tL / (1.0 + exp(-ta_sig * (iy - tshift_y))) + tshift_x);
                }
            }

            // ROS化
            geometry_msgs::Point pt;
            ob_point.clear();
            for (int i=0; i<x.size(); i++) {
                pt.x = x[i];
                pt.y = y[i];
                pt.z = 0.15;
                ob_point.push_back(pt);
            }
            type1_pose.clear();
            type2_pose.clear();
            type3_pose.clear();
            type4_pose.clear();
            type5_pose.clear();
            for (int i=0; i<func_2d_x.size(); i++) {
                pt.x = func_1d_x[i];
                pt.y = func_1d_y[i];
                pt.z = 0.13;
                type1_pose.push_back(pt);
                pt.x = func_2d_x[i];
                pt.y = func_2d_y[i];
                pt.z = 0.10;
                type2_pose.push_back(pt);
                pt.x = func_t2d_x[i];
                pt.y = func_t2d_y[i];
                pt.z = 0.07;
                type3_pose.push_back(pt);
                pt.x = func_sig_x[i];
                pt.y = func_sig_y[i];
                pt.z = 0.04;
                type4_pose.push_back(pt);
                pt.x = func_tsig_x[i];
                pt.y = func_tsig_y[i];
                pt.z = 0.01;
                type5_pose.push_back(pt);
            }
            if      (ob_point.size() == 1) ob_colors = {red};
            else if (ob_point.size() == 2) ob_colors = {purple, red};
            else if (ob_point.size() == 3) ob_colors = {blue, purple, red};
            type1_colors.resize(type1_pose.size(), cyan);
            type2_colors.resize(type2_pose.size(), green);
            type3_colors.resize(type3_pose.size(), yellow);
            type4_colors.resize(type4_pose.size(), orange);
            type5_colors.resize(type5_pose.size(), pink);

            if (mode == 1) {
                if (type1_check) printf("TYPE1(CYAN)   = O\n");
                else             printf("TYPE1(CYAN)   = X\n");
                if (type2_check) printf("TYPE2(GREEN)  = O\n");
                else             printf("TYPE2(GREEN)  = X\n");
                if (type3_check) printf("TYPE3(YELLOW) = O\n");
                else             printf("TYPE3(YELLOW) = X\n");
                if (type4_check) printf("TYPE4(ORANGE) = O\n");
                else             printf("TYPE4(ORANGE) = X\n");
                if (type5_check) printf("TYPE5(PINK)   = O\n");
                else             printf("TYPE5(PINK)   = X\n");
                printf("\n");
            }
        }
        // 描画
        void viewer() {
            ros::spinOnce();
            ros::Rate r(5);
            while (ros::ok()) {
                pub_image.publish(base_map);
                if (x.size() != 0) pub_marker_ob.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::POINTS, "map", "obstacle", base_pose, ob_point, ob_size, ob_colors, ros::Duration(0.4) ) );
                if (no_path_del) {
                    if (type1_check) pub_marker_type1.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::LINE_STRIP, "map", "type1", base_pose, type1_pose, line_size, type1_colors, ros::Duration(0.4) ) );
                    if (type2_check) pub_marker_type2.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::LINE_STRIP, "map", "type2", base_pose, type2_pose, line_size, type2_colors, ros::Duration(0.4) ) );
                    if (type3_check) pub_marker_type3.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::LINE_STRIP, "map", "type3", base_pose, type3_pose, line_size, type3_colors, ros::Duration(0.4) ) );
                    if (type4_check) pub_marker_type4.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::LINE_STRIP, "map", "type4", base_pose, type4_pose, line_size, type4_colors, ros::Duration(0.4) ) );
                    if (type5_check) pub_marker_type5.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::LINE_STRIP, "map", "type5", base_pose, type5_pose, line_size, type5_colors, ros::Duration(0.4) ) );
                } else {
                    pub_marker_type1.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::LINE_STRIP, "map", "type1", base_pose, type1_pose, line_size, type1_colors, ros::Duration(1) ) );
                    pub_marker_type2.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::LINE_STRIP, "map", "type2", base_pose, type2_pose, line_size, type2_colors, ros::Duration(1) ) );
                    pub_marker_type3.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::LINE_STRIP, "map", "type3", base_pose, type3_pose, line_size, type3_colors, ros::Duration(1) ) );
                    pub_marker_type4.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::LINE_STRIP, "map", "type4", base_pose, type4_pose, line_size, type4_colors, ros::Duration(1) ) );
                    pub_marker_type5.publish( marker_lib.makeMarkerList( visualization_msgs::Marker::LINE_STRIP, "map", "type5", base_pose, type5_pose, line_size, type5_colors, ros::Duration(1) ) );
                }
                ros::spinOnce();
                r.sleep();
            }
        }
        void set_base_map(std::string img_path) {
            // 画像をグレースケールで読み込む
            cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

            if (img.empty()) {
                ROS_ERROR("画像の読み込みに失敗しました！");
                return;
            }

            int height = img.rows;
            int width = img.cols;

            // OccupancyGridメッセージの準備
            base_map.header.frame_id = "map";
            base_map.info.resolution = 0.006;  // 必要に応じて解像度を調整

            base_map.info.width = width;
            base_map.info.height = height;

            // Poseの初期化
            base_map.info.origin.position.x = 3.8776;
            base_map.info.origin.position.y = 2.90286;
            base_map.info.origin.position.z = 0.0;

            base_map.info.origin.orientation.x = 0.0;
            base_map.info.origin.orientation.y = 0.0;
            base_map.info.origin.orientation.z = 1.0;
            base_map.info.origin.orientation.w = 0.0;

            // 画像ピクセルを占有グリッドデータに変換
            std::vector<int8_t> occupancy_data(height * width, 0);

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    // ピクセル値を占有値に変換 (0 = free, 100 = occupied)
                    if (img.at<uchar>(y, x) > 127) {
                        occupancy_data[y * width + x] = 0;  // Free space
                    } else {
                        occupancy_data[y * width + x] = 100;  // Occupied space
                    }
                }
            }

            base_map.data = occupancy_data;
        }
        static double equation(const std::vector<double> &tx, std::vector<double> &grad, void *data) {
            Trajectory* obj = static_cast<Trajectory*>(data);

            double term1 = (obj->y[0] - obj->y[1]) / (1. / (1. + std::exp(-tx[0] * (obj->x[0] - obj->x[1]))) - 0.5);
            double term2 = (obj->y[2] - obj->y[1]) / (1. / (1. + std::exp(-tx[0] * (obj->x[2] - obj->x[1]))) - 0.5);
            return term1 - term2; // 方程式の値を返す
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "predict_traj");
    Trajectory traj;
}