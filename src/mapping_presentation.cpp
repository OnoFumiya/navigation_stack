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
            position_x = odom.pose.pose.position.x + misalignment.position.x;
            position_y = odom.pose.pose.position.y + misalignment.position.y;
            position_z = odom.pose.pose.position.z + misalignment.position.z;
            sita = (2*(acos(odom.pose.pose.orientation.w + misalignment.orientation.w)))*((odom.pose.pose.orientation.z + misalignment.orientation.z)*(odom.pose.pose.orientation.w + misalignment.orientation.w))/(std::fabs((odom.pose.pose.orientation.z + misalignment.orientation.z)*(odom.pose.pose.orientation.w + misalignment.orientation.w)));
            if ((std::fabs(sita)) > M_PI)
            {
                sita = (2*M_PI - std::fabs(sita))*(((odom.pose.pose.orientation.z + misalignment.orientation.z)*(odom.pose.pose.orientation.w + misalignment.orientation.w))/(std::fabs((odom.pose.pose.orientation.z + misalignment.orientation.z)*(odom.pose.pose.orientation.w + misalignment.orientation.w))));
            }
        }
    public:
        geometry_msgs::Pose misalignment;
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
                misalignment.position.x = 0.0;
                misalignment.position.y = 0.0;
                misalignment.position.z = 0.0;
                misalignment.orientation.x = 0.0;
                misalignment.orientation.y = 0.0;
                misalignment.orientation.z = 0.0;
                misalignment.orientation.w = 0.0;
                ros::spinOnce();
                if ((position_x != 100) || (position_y != 100))
                {
                    break;
                }
            }
        }
};


class OBSTACLE_DIST
{
    private:
        ros::Subscriber sub_dist;
        ROBOT_POSITION robot_position;
        float maxdist, mindist;
        int i;
        void callback_obstacle(const sensor_msgs::LaserScan &ob)
        {
            range.clear();
            range_point.clear();
            float a;
            range_angle_increment = ob.angle_increment;
            maxdist = ob.range_max;
            mindist = ob.range_min;
            for (i=0; i<ob.ranges.size(); i++)
            {
                if ((mindist <= ob.ranges[i]) && (ob.ranges[i] <= maxdist))
                {
                    a = (robot_position.sita + ob.angle_min + range_angle_increment*i);
                    range.push_back(ob.ranges[i]);
                    range_point.push_back({robot_position.position_x + (ob.ranges[i]*(cos(a))), robot_position.position_y + (ob.ranges[i]*(sin(a))), 0, a});
                }
            }
            range_size = range.size();
        }
    public:
        std::vector<float> range;
        std::vector<std::vector<float>> range_point;
        long range_size = std::numeric_limits<int>::max();
        float range_angle_increment;
        OBSTACLE_DIST()
        {
            ros::NodeHandle node;
            sub_dist = node.subscribe("/scan", 10, &OBSTACLE_DIST::callback_obstacle, this);
            get_dist();
        }
        void get_dist()
        {
            ros::spinOnce();
            while(1)
            {
                ros::spinOnce();
                if (range_size == range.size())
                {
                    ros::spinOnce();
                    break;
                }
            }
            ros::spinOnce();
        }
};


class MOVE_CLASS
{
    private:
        ros::Publisher pub_twist;
    public:
        MOVE_CLASS()
        {
            ros::NodeHandle node;
            pub_twist = node.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 10);
        }
        void straight_and_turn_time(float u_vel, float u_ang, float dt)
        {
            ros::Rate rate(5.0);
            geometry_msgs::Twist vel;
            struct timeval t;
            int i = 0;
            while (1)
            {
                if (dt-i >= 1)
                {
                    i++;
                }
                else
                {
                    break;
                }
            }
            long udt = (dt-i)*(pow(10,6));
            dt = i;
            vel.linear.x = u_vel;
            vel.angular.z = u_ang*0.99;
            gettimeofday(&t, NULL);    //時間を取得
            long sec_0 = t.tv_sec , sec;     //整数部分の秒(s)
            long usec_0 = t.tv_usec, usec;   //小数以下第6位までの((10**(-6))s)
            ros::spinOnce();
            while (ros::ok())
            {
                ros::spinOnce();
                pub_twist.publish(vel);
                gettimeofday(&t, NULL);
                sec = t.tv_sec;     //整数部分の秒(s)
                usec = t.tv_usec;   //小数以下第6位までの((10**(-6))s)
                if (((sec - sec_0) >= dt) || (dt == 0))
                {
                    if ((udt == 0) || (((usec - usec_0) >= 0) && (usec - usec_0) >= udt) || (((usec - usec_0) < 0) && ((usec+(pow(10,6))-usec_0) >= udt)))
                    {
                        break;
                    }
                }
            }
        }
        void stop_vel()
        {
            geometry_msgs::Twist vel;
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
            struct timeval t;
            gettimeofday(&t, NULL);    //時間を取得
            long sec_0 = t.tv_sec, sec;     //整数部分の秒(s)
            while (1)
            {
                gettimeofday(&t, NULL);    //時間を取得
                sec = t.tv_sec;     //整数部分の秒(s)
                ros::spinOnce();
                pub_twist.publish(vel);
                if ((sec-sec_0) > 3)
                {
                    break;
                }
            }
        }
        void go(float st, float si, float t)
        {
            ros::spinOnce();
            straight_and_turn_time(st, si, t);
            stop_vel();
            ros::spinOnce();
        }
};


class PRESENTATION
{
    private:
        ros::Publisher pub_global_map;
        ros::Publisher pub_astar_node;
    public:
        GRIDDING gridding;
        ROBOT_POSITION robot_position;
        OBSTACLE_DIST obstacle_dist;
        navigation_stack::MapInformation map;
        MOVE_CLASS move_class;
        std::vector<float> pose;
        PRESENTATION()
        {
            ros::NodeHandle node;
            pub_global_map = node.advertise<navigation_stack::MapInformation>("/global_map", 10);
            pub_astar_node = node.advertise<navigation_stack::MapInformation>("/astar_node", 10);
            global_map();
            // a_star();
            move_class.go(0.0, 0.5,5.4);
            move_class.go(0.2, 0.0,6.5);
            global_map();
        }
        void global_map()
        {
            geometry_msgs::Vector3 vec;
            geometry_msgs::Vector3 vect;
            bool frag;
            pose.resize(4);
            pose[0] = robot_position.position_x;
            pose[1] = robot_position.position_y;
            pose[2] = 0.0;
            pose[3] = robot_position.sita;
            vec.x = ss(obstacle_dist.range_point[0][0], gridding.size);
            vec.y = ss(obstacle_dist.range_point[0][1], gridding.size);
            vec.z = 0.0;
            map.cost.push_back(vec);
            vect.x = ss(pose[0],gridding.size);
            vect.y = ss(pose[1],gridding.size);
            vect.z = 0.0;
            map.clearly.push_back(vect);
            for (int i=0; i<obstacle_dist.range.size(); i++)
            {
                frag = true;
                for (int j=0; j<map.cost.size(); j++)
                {
                    if ((((map.cost[j].x - (gridding.size/2)) <= obstacle_dist.range_point[i][0]) && (obstacle_dist.range_point[i][0] < ((map.cost[j].x + (gridding.size/2))))) && (((map.cost[j].y - (gridding.size/2)) <= obstacle_dist.range_point[i][1]) && (obstacle_dist.range_point[i][1] < ((map.cost[j].y + (gridding.size/2))))))
                    {
                        frag = false;
                        break;
                    }
                }
                if (frag)
                {
                    vec.x = ss(obstacle_dist.range_point[i][0], gridding.size);
                    vec.y = ss(obstacle_dist.range_point[i][1], gridding.size);
                    map.cost.push_back(vec);
                    global_clearly_map(i);
                }
            }
            ros::spinOnce();
            struct timeval t;
            gettimeofday(&t, NULL);    //時間を取得
            long sec_0 = t.tv_sec, sec;     //整数部分の秒(s)
            while (1)
            {
                gettimeofday(&t, NULL);    //時間を取得
                sec = t.tv_sec;     //整数部分の秒(s)
                ros::spinOnce();
                pub_global_map.publish(map);
                if ((sec-sec_0) > 3)
                {
                    break;
                }
            }
            ros::spinOnce();
        }
        void global_clearly_map(int ii)
        {
            geometry_msgs::Vector3 vecto;
            bool frag;
            float a,b;
            if (((M_PI/4) < std::fabs(obstacle_dist.range_point[ii][3])) && (std::fabs(obstacle_dist.range_point[ii][3]) < (3*M_PI/4)))
            {
                for (int j=1; j<((std::fabs(obstacle_dist.range_point[ii][1] - pose[1]))/gridding.size); j++)
                {
                    a = ss(pose[0]+((j*gridding.size/tan(obstacle_dist.range_point[ii][3]))*((std::fabs(obstacle_dist.range_point[ii][1]-pose[1]))/(obstacle_dist.range_point[ii][1]-pose[1]))),gridding.size,false);
                    b = ss(pose[1]+((j*gridding.size                                      )*((std::fabs(obstacle_dist.range_point[ii][1]-pose[1]))/(obstacle_dist.range_point[ii][1]-pose[1]))),gridding.size,false);
                    frag = true;
                    for (int k=0; k<map.clearly.size(); k++)
                    {
                        if ((((map.clearly[k].x-(gridding.size/2)) <= a) && (a < (map.clearly[k].x+(gridding.size/2)))) && (((map.clearly[k].y-(gridding.size/2)) <= b) && (b < (map.clearly[k].y+(gridding.size/2)))))
                        {
                            frag = false;
                            break;
                        }
                    }
                    if (frag)
                    {
                        vecto.x = a;
                        vecto.y = b;
                        vecto.z = 0.0;
                        map.clearly.push_back(vecto);
                    }
                }
            }
            else
            {
                for (int j=0; j<((std::fabs(obstacle_dist.range_point[ii][0]-pose[0]))/gridding.size); j++)
                {
                    a = ss(pose[0]+((j*gridding.size                                      )*((std::fabs(obstacle_dist.range_point[ii][0]-pose[0]))/(obstacle_dist.range_point[ii][0]-pose[0]))),gridding.size,false);
                    b = ss(pose[1]+((j*gridding.size*tan(obstacle_dist.range_point[ii][3]))*((std::fabs(obstacle_dist.range_point[ii][0]-pose[0]))/(obstacle_dist.range_point[ii][0]-pose[0]))),gridding.size,false);
                    frag = true;
                    for (int k=0; k<map.clearly.size(); k++)
                    {
                        if ((((map.clearly[k].x-(gridding.size/2)) <= a) && (a < (map.clearly[k].x+(gridding.size/2)))) && (((map.clearly[k].y-(gridding.size/2)) <= b) && (b < (map.clearly[k].y+(gridding.size/2)))))
                        {
                            frag = false;
                            break;
                        }
                    }
                    if (frag)
                    {
                        vecto.x = a;
                        vecto.y = b;
                        vecto.z = 0.0;
                        map.clearly.push_back(vecto);
                    }
                }
            }
        }
        // void a_star()
        // {
        //     bool f=false;
        //     while (ros::ok())
        //     {
        //         f=false;
        //         for(int j=0; j<map.clearly.size(); j++)
        //         {
        //             if ((std::fabs((pose[0] + gridding.size) - map.clearly[j].x)  < 0.05) && (std::fabs((pose[1] + gridding.size) - map.clearly[j].y) < 0.05))
        //             {
        //                 pose[0] = (ss(pose[0], gridding.size) + gridding.size, gridding.size);
        //                 pose[1] = (ss(pose[1], gridding.size) + gridding.size, gridding.size);
        //                 f = true;
        //             }
        //         }
        //         if (f==false)
        //         {
        //             break;
        //         }
        //     }
        //     navigation_stack::MapInformation node;
        //     geometry_msgs::Vector3 n;
        //     n.x = pose[0];
        //     n.y = pose[1];
        //     n.z = 0.0;
        //     node.cost.push_back(n);
        //     printf("%f,  %f\n",n.x,n.y);
        //     // int si;
        //     // bool t,b,l,r;
        //     // for (int i=0; i<map.clearly.size(); i++)
        //     // {
        //     //     t = false;
        //     //     b = false;
        //     //     l = false;
        //     //     r = false;
        //     //     for (int j=0; j<4; j++)
        //     //     {
        //     //         if (j==0)
        //     //         {
        //     //             n.x = ss((map.clearly[i].x), gridding.size);
        //     //             n.y = ss((map.clearly[i].y + gridding.size), gridding.size);
        //     //         }
        //     //         else if (j==1)
        //     //         {
        //     //             n.x = ss((map.clearly[i].x), gridding.size);
        //     //             n.y = ss((map.clearly[i].y - gridding.size), gridding.size);
        //     //         }
        //     //         else if (j==2)
        //     //         {
        //     //             n.x = ss((map.clearly[i].x - gridding.size), gridding.size);
        //     //             n.y = ss((map.clearly[i].y), gridding.size);
        //     //         }
        //     //         else
        //     //         {
        //     //             n.x = ss((map.clearly[i].x + gridding.size), gridding.size);
        //     //             n.y = ss((map.clearly[i].y), gridding.size);
        //     //         }
        //     //         for (int k=0; k<map.clearly.size(); k++)
        //     //         {
        //     //             if ((i!=j) && ((((map.clearly[k].x -(gridding.size/2)) <= n.x) && (n.x < (map.clearly[k].x + (gridding.size/2)))) && (((map.clearly[k].y -(gridding.size/2)) <= n.y) && (n.y < (map.clearly[k].y + (gridding.size/2))))))
        //     //             {
        //     //                 if (j==0)
        //     //                 {
        //     //                     t = true;
        //     //                 }
        //     //                 else if (j==1)
        //     //                 {
        //     //                     b = true;
        //     //                 }
        //     //                 else if (j==2)
        //     //                 {
        //     //                     l = true;
        //     //                 }
        //     //                 else
        //     //                 {
        //     //                     r = true;
        //     //                 }
        //     //                 break;
        //     //             }
        //     //         }
        //     //     }
        //     //     if ((t==false) || (b==false) || (l==false) || (r==false))
        //     //     {
        //     //         node.cost.push_back(n);
        //     //     }
        //     // }
        //     ros::spinOnce();
        //     pub_astar_node.publish(node);
        //     ros::spinOnce();
        // }
        float ss(float s, float g, bool f=true)
        {
            float r = s - (((float)(s/g) - (int)(s/g))*g);
            if ((s<0) && (f))
            {
                r-=g;
            }
            r += (g/2);
            return r;
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_presentation");
    PRESENTATION presentation;
    ros::spin();
}