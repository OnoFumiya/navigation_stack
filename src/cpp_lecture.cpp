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
#include <nlopt.hpp>
#include <sys/time.h>
#include <fstream>
#include <matplotlib-cpp/matplotlibcpp.h>
#include <navigation_stack/MapInformation.h>
#include <navigation_stack/PathPoint.h>
#include <Eigen/Dense>


double equation(const std::vector<double> &sx, std::vector<double> &grad, void *my_func_data) {
    double y12, y32, x12, x32;
    y12 = -2.7;
    y32 = 2.6;
    x12 = -3.5;
    x32 = 1.4;
    double term1 = (y12) / (1. / (1. + exp(-sx[0] * (x12))) - 0.5);
    double term2 = (y32) / (1. / (1. + exp(-sx[0] * (x32))) - 0.5);
    return std::abs(term1 - term2); // 方程式の値を返す
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_lecture");
    nlopt::opt opt(nlopt::LN_COBYLA, 1); // 最適化手法を選択

    // 最適化問題を設定
    opt.set_min_objective(equation, nullptr);
    opt.set_xtol_rel(1e-6); // 収束条件
    std::vector<double> sx = {0.1}; // 初期推定値

    double minf; // 最小値
    nlopt::result result = opt.optimize(sx, minf);

    if (result < 0) {
        std::cerr << "最適化失敗" << std::endl;
    } else {
        std::cout << "解: " << sx[0] << std::endl;
    }
}