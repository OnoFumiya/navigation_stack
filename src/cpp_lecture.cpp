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

class Cpp_Lecture
{
    private:
        double a, b, c;
    public:
        Cpp_Lecture() {
            loop();
        }
        void loop() {
            a = 1.;
            b = 4.;
            c = 6.6666;
            std::vector<geometry_msgs::Point> tabc(3);
            tabc[0].x = a;
            tabc[1].y = b;
            tabc[2].z = c;
            check(tabc);
            printf("(a , b , c ) = (%f, %f, %f)\n", a , b , c );
            printf("(aa, bb, cc) = (%f, %f, %f)\n", tabc[0].x, tabc[1].y, tabc[2].z);
        }
        void check(std::vector<geometry_msgs::Point> &tt) {//double &ta, double &tb, double &tc
            a = a + b + c;
            b = a * b * c;
            c = 0.;
            tt[0].x = tt[0].x + tt[1].y + tt[2].z;
            tt[1].y = tt[0].x * tt[1].y * tt[2].z;
            tt[2].z = 0.;
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_lecture");
    Cpp_Lecture cpp_lecture;
}