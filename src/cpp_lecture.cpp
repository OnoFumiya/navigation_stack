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


// class Sigmoid {
//     private:
//         double L, k, x_c;
//         double x_diff, y_diff;
//     public:
//         // シグモイド関数
//         double sigmoid(double x) {
//             return L / (1.0 + exp(-k * (x - x_c)));
//         }

//         // 最小二乗法を用いてパラメータをフィッティングする関数（ここでは簡易なアプローチ）
//         void fitSigmoid(const std::vector<double>& x, const std::vector<double>& y) {
//             L = y[0];
//             for (int i=0; i<3; i++) {
//                 if (L < y[i])
//             }
//             L = *std::max_element(y.begin(), y.end()); // yの最大値をLと仮定
//             k = 1.0; // 初期値として1.0
//             x_c = x[1];
//         }
// };


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_lecture");
    std::vector<double> func_2d_x, func_2d_y;
    std::vector<double> func_sig_x, func_sig_y;
    std::vector<double> x({0,1,2});
    std::vector<double> y({0,1,2});

    // 放物線
    double a, b, c;
    a = (x[0]*(y[2] - y[1]) + x[1]*(y[0] - y[2]) + x[2]*(y[1] - y[0])) / ((x[0] - x[1]) * (x[0] - x[2]) * (x[1] - x[2]));
    b = ((std::pow(x[1], 2) - std::pow(x[2], 2))*y[0] + (std::pow(x[2], 2) - std::pow(x[0], 2))*y[1] + (std::pow(x[0], 2) - std::pow(x[1], 2))*y[2]) / ((x[0] - x[1]) * (x[0] - x[2]) * (x[2] - x[1]));
    c = y[0] - a*std::pow(x[0], 2) -b*x[0];

    // シグモイド曲線
    double L, k, x_c, x_diff, y_diff;
    x_c = x[1];
    // x_c = x[2]/2 + x[0]/2;
    // k = (y[1] - y[0]) / (x[1] - x[0]);
    k = 1.0;
    L = y[2];
    // if (y[0] < y[2]) {
    //     L = y[2] - y[0];
    //     y_diff = y[0];
    // }
    // else {
    //     L = y[0] - y[2];
    //     y_diff = y[2];
    // }
    // if (x[0] < x[2]) x_diff = x[0];
    // else             x_diff = x[2];


    // 描画
    for (int i=0; i<300; i++) {
        double ix = ((double)i/100);
        func_2d_x.push_back(ix);
        func_2d_y.push_back(a*std::pow(ix, 2) + b*ix + c);

        func_sig_x.push_back(ix);
        func_sig_y.push_back(L / (1.0 + exp(-k * ((ix) - x_c))));
    }

    // matplotlibcpp::plot(func_2d_x,func_2d_y, "-y");
    matplotlibcpp::plot(func_sig_x,func_sig_y, "--b");
    matplotlibcpp::plot(x, y, "ob");
    matplotlibcpp::show();

}