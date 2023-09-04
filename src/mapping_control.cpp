#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/Vector3.h>
// #include <nav_msgs/Odometry.h>
// #include <sensor_msgs/LaserScan.h>
// #include <vector>
// #include <string>
// #include <iostream>
// #include <limits>
// #include <math.h>
// #include <typeinfo>
// #include <sys/time.h>

class ALL_CONTROL
{
    private:
        ros::Publisher pub_start;
        ros::Subscriber sub_end;
        void callback_endjudge(const std_msgs::Bool &get_judge)
        {
            judge.data = get_judge.data;
        }
    public:
        std_msgs::Bool judge;
        ALL_CONTROL()
        {
            ros::NodeHandle node;
            sub_end = node.subscribe("/end", 10, &ALL_CONTROL::callback_endjudge, this);
            pub_start = node.advertise<std_msgs::Bool>("/start", 10);
            judgement();
        }
        void judgement()
        {
            std_msgs::Bool start;
            start.data = true;
            ros::spinOnce();
            pub_start.publish(start);
            ros::spinOnce();
            judge.data = false;
            while (ros::ok())
            {
                if (judge.data)
                {
                    ros::spinOnce();
                    break;
                }
                ros::spinOnce();
            }
            ros::spinOnce();
            savemap();
        }
        void savemap()
        {
            char mapname[100];
            printf("\nMapping end!!\n");
            printf("\nPlease input the creating map name : ");
            scanf("%s", mapname);
            printf("\n");
            printf("\nSuccess creating ”%s” map\n",mapname);
            ros::spinOnce();
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_control");
    ros::spinOnce();
    ALL_CONTROL all_control;
    ros::spin();
}
