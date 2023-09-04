#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <iostream>
#include <limits>
#include <math.h>
#include <typeinfo>
#include <sys/time.h>
#include <matplotlib-cpp/matplotlibcpp.h>
#include <navigation_stack/MapInformation.h>
#include <geometry_msgs/Pose.h>


class GRIDDING
{
    public:
        float size = 0.05;
};


class ROBOT_POSITION
{   private:
        ros::Subscriber sub_odom;
        void callback_odom(const nav_msgs::Odometry &odom)
        {
            position_x = odom.pose.pose.position.x + missed.position.x;
            position_y = odom.pose.pose.position.y + missed.position.y;
            position_z = odom.pose.pose.position.z + missed.position.z;
            sita = (2*(acos(odom.pose.pose.orientation.w + missed.orientation.w)))*((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w))/(std::fabs((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w)));
            if ((std::fabs(sita)) > M_PI)
            {
                sita = (2*M_PI - std::fabs(sita))*(((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w))/(std::fabs((odom.pose.pose.orientation.z + missed.orientation.z)*(odom.pose.pose.orientation.w + missed.orientation.w))));
            }
        }
    public:
        geometry_msgs::Pose missed;
        // std::vector<float> pose_miss{0.0,0.0,0.0};
        // float sita_missed = 0.0;
        float position_x = 100.0, position_y = 100.0, position_z = 0.0, sita;
        ROBOT_POSITION()
        {
            ros::NodeHandle node;
            sub_odom = node.subscribe("/odom", 10, &ROBOT_POSITION::callback_odom, this);
            get_point();
        }
        void get_point()
        {
            ros::spinOnce();
            while (1)
            {
                ros::spinOnce();
                if ((position_x != 100) || (position_y != 100))
                {
                    break;
                }
            }
        }
};


class MAP_LOCALIZATION
{
    private:
        ros::Publisher pub_local_map;
        ros::Subscriber sub_global_map;
        ros::Subscriber sub_dist;
        ROBOT_POSITION robot_position;
        bool frag = false;
        void callback_map(const navigation_stack::MapInformation &global_getmap)
        {
            costmap.clear();
            for (int i=0; i<global_getmap.cost.size(); i++)
            {
                costmap.push_back(global_getmap.cost[i]);
            }
            frag = true;
        }
        void callback_obstacle(const sensor_msgs::LaserScan &ob)
        {
            range.clear();
            range_point.clear();
            for (int i=0; i<ob.ranges.size(); i++)
            {
                if ((ob.range_min <= ob.ranges[i]) && (ob.ranges[i] <= ob.range_max))
                {
                    range.push_back(ob.ranges[i]);
                    range_point.push_back({robot_position.position_x + (ob.ranges[i]*(cos(robot_position.sita + ob.angle_min + ob.angle_increment*i))), robot_position.position_y + (ob.ranges[i]*(sin(robot_position.sita + ob.angle_min + ob.angle_increment*i))), 0, (robot_position.sita + ob.angle_min + ob.angle_increment*i)});
                }
            }
            range_size = range.size();
        }
    public:
        MAP_LOCALIZATION()
        {
            ros::NodeHandle node;
            pub_local_map = node.advertise<geometry_msgs::Pose>("/localization_missed", 10);
            sub_global_map = node.subscribe("/global_map", 10, &MAP_LOCALIZATION::callback_map, this);
            sub_dist = node.subscribe("/scan", 10, &MAP_LOCALIZATION::callback_obstacle, this);
            get_dist();
            localization();
        }
        // navigation_stack::MapInformation local_map;
        GRIDDING gridding;
        std::vector<geometry_msgs::Vector3> costmap;
        // geometry_msgs::Pose missed;
        geometry_msgs::Vector3 vec;
        std::vector<float> range;
        std::vector<std::vector<float>> range_point;
        long range_size = std::numeric_limits<int>::max();
        void get_dist()
        {
            ros::spinOnce();
            while(ros::ok())
            {
                ros::spinOnce();
                if (range_size == range.size())
                {
                    ros::spinOnce();
                    break;
                }
            }
            ros::spinOnce();
            printf("aaa\n");
        }
        void localization()
        {
            ros::spinOnce();
            std::vector<float> point(4);
            std::vector<float> pose_miss{0.0,0.0,0.0};
            float sita_missed = 0.0;
            ros::spinOnce();
            vec.z = pose_miss[2];
            ros::spinOnce();
            while (ros::ok())
            {
                ros::spinOnce();
                if (frag)
                {
                    ros::spinOnce();
                    break;
                }
                vec.x = pose_miss[0];
                vec.y = pose_miss[1];
                sita_missed = 0.0;
                missed_publisher(vec, sita_missed);
                ros::spinOnce();
            }
            ros::spinOnce();
            while (ros::ok())
            {
                point[0] = robot_position.position_x;
                point[1] = robot_position.position_y;
                point[2] = robot_position.position_z;
                point[3] = robot_position.sita;
                pose_miss = localization_pose(point, pose_miss, sita_missed, costmap, range_point);
                vec.x = pose_miss[0];
                vec.y = pose_miss[1];
                vec.z = pose_miss[2];
                missed_publisher(vec, sita_missed);
                ros::spinOnce();
            }
        }
        void missed_publisher(geometry_msgs::Vector3 position_pub, float sita_pub)
        {
            robot_position.missed.position.x = position_pub.x;
            robot_position.missed.position.y = position_pub.y;
            robot_position.missed.position.z = position_pub.z;
            if (sita_pub <= 0.0)
            {
                sita_pub += 2*M_PI;
            }
            robot_position.missed.orientation.x = cos(sita_pub/2);
            robot_position.missed.orientation.y = 0.0;
            robot_position.missed.orientation.z = 0.0;
            robot_position.missed.orientation.w = sin(sita_pub/2);
            pub_local_map.publish(robot_position.missed);
            ros::spinOnce();
        }
        // float localization_sita(std::vector<float> me, std::vector<float> fixpose, float fixsita, std::vector<std::vector<float>> ob_global, std::vector<float> obran)
        // {
        //     // me[0] += fixpose[0];
        //     // me[1] += fixpose[1];
        //     // me[3] += fixsita;
        //     std::vector<std::vector<float>> ob_local;
        //     std::vector<float> oblocal_to_obglobal;
        //     float sita_miss = (M_PI/180)*(3);
        //     float sita_grid = (M_PI/180)*(0.1);
        //     float pose_miss = 0.1;
        //     float oblocal_to_obglobal_min;
        //     float var_stack = std::numeric_limits<float>::max();
        //     float var, ave;
        //     float new_fixsita;
        //     bool min_ok_frag;
        //     for (float i=(me[2]-sita_miss); i<=(me[2]+sita_miss); i+=sita_grid)
        //     {
        //         ob_local.clear();
        //         for (int j=0; j<obran.size(); j++)
        //         {
        //             ob_local.push_back({me[0] + obran[j]*cos(i), me[1] + obran[j]*sin(i)});
        //         }
        //         oblocal_to_obglobal.clear();
        //         for (int j=0; j<ob_local.size(); j++)
        //         {
        //             oblocal_to_obglobal_min = std::numeric_limits<float>::max();
        //             min_ok_frag = false;
        //             for (int k=0; k<ob_global.size(); k++)
        //             {
        //                 if ((sqrt((pow((ob_local[j][0]-ob_global[k][0]),2))+(pow((ob_local[j][1]-ob_global[k][1]),2)))) <= ((sqrt(2))*pose_miss))
        //                 // if ((sqrt((pow((ob_local[j][0]-ob_global[k][0]),2))+(pow((ob_local[j][1]-ob_global[k][1]),2)))) > (2*pose_miss))
        //                 // if (1)
        //                 {
        //                     continue;
        //                 }
        //                 else if ((sqrt((pow((ob_local[j][0]-ob_global[k][0]),2))+(pow((ob_local[j][1]-ob_global[k][1]),2)))) < oblocal_to_obglobal_min)
        //                 {
        //                     oblocal_to_obglobal_min = sqrt((pow((ob_local[j][0]-ob_global[k][0]),2))+(pow((ob_local[j][1]-ob_global[k][1]),2)));
        //                     min_ok_frag = true;
        //                 }
        //             }
        //             if (min_ok_frag)
        //             {
        //                 oblocal_to_obglobal.push_back(oblocal_to_obglobal_min);
        //             }
        //         }
        //         var = 0;
        //         ave = 0;
        //         for (int j=0; j<oblocal_to_obglobal.size(); j++)
        //         {
        //             ave += ((oblocal_to_obglobal[j])/(oblocal_to_obglobal.size()));
        //         }
        //         for (int j=0; j<oblocal_to_obglobal.size(); j++)
        //         {
        //             var += ((pow((oblocal_to_obglobal[j]-ave),2))/**(oblocal_to_obglobal.size())*/);
        //         }
        //         if (var < var_stack)
        //         {
        //             var_stack = var;
        //             new_fixsita = i;
        //         }
        //     }
        //     fixsita += (new_fixsita - me[2]);
        //     return fixsita;
        // }
        std::vector<float> localization_pose(std::vector<float> me, std::vector<float> fixpose, float fixsita, std::vector<geometry_msgs::Vector3> ob_global, std::vector<std::vector<float>> ob_local)
        {
            std::vector<float> oblocal_to_obglobal;
            std::vector<float> new_fixpose{0,0};
            float pose_miss = 0.2;
            float grid = gridding.size;
            float oblocal_to_obglobal_min;
            int score, is, js;
            int score_stack = 0;
            for (float i=((-1)*pose_miss); i<=pose_miss; i+=grid)
            {
                for (float j=((-1)*pose_miss); j<=pose_miss; j+=grid)
                {
                    score = 0;
                    for (int l=0; l<ob_local.size(); l++)
                    {
                        for (int m=0; m<ob_global.size(); m++)
                        {
                            if ((((ob_global[m].x-(grid/2)) <= (ob_local[l][0]+i)) && ((ob_local[l][0]+i) < (ob_global[m].x+(grid/2)))) && (((ob_global[m].y-(grid/2)) <= (ob_local[l][1]+j)) && ((ob_local[l][1]+j) < (ob_global[m].y+(grid/2)))))
                            {
                                score++;
                                break;
                            }
                        }
                    }
                    if (score_stack < score)
                    {
                        score_stack = score;
                        new_fixpose[0] = i;
                        new_fixpose[1] = j;
                    }
                }
            }
            fixpose[0] += new_fixpose[0];
            fixpose[1] += new_fixpose[1];
            return fixpose;
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_localization");
    MAP_LOCALIZATION map_localization;
    ros::spin();
}