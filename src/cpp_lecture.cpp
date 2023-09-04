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
#include <cmath>
#include <math.h>
#include <typeinfo>
#include <sys/time.h>
#include <matplotlib-cpp/matplotlibcpp.h>
#include <navigation_stack/MapInformation.h>



class GRIDDING
{
    public:
        float size = 0.05;
        float arg_size = 1/size;
        float float_to_grid(float s, bool f=true)  // 適当な値をgrid幅に矯正する関数
        {
            float r = s - (((float)(s/size) - (int)(s/size))*size);
            if ((s<0) && (f))
            {
                r-=size;
            }
            r += (size/2);
            return r;
        }
        int float_to_int(float s, bool f=true)  // grid幅の値を0を基準にした格納番号(int型)に変換する関数
        {
            int r = s*arg_size;
            if ((s<0) && (f))
            {
                r--;
            }
            return r;
        }
        float int_to_grid(int s)  // float_to_intの逆をする
        {
            return (float)((s/arg_size) + (1/(2*arg_size)));
        }
};



class DIJKSTRA_LECTURE
{
    private:
        int plot_size;
        int zero_point;
        int limit_point[4];
        std::vector<int> vector_1d;
        std::vector<std::vector<int>> vector_2d;
        std::vector<std::vector<int>> vector_2d_dijkstra_cost;
    public:
        DIJKSTRA_LECTURE()
        {
            plot_size = (int)((sqrt(std::numeric_limits<int>::max()))/2);
            zero_point = (int)(plot_size/2);
            vector_1d.resize(plot_size,-1);
            vector_2d.resize(plot_size,vector_1d);
            vector_1d.clear();
            vector_1d.resize(plot_size,plot_size*2);
            vector_2d_dijkstra_cost.resize(plot_size,vector_1d);
            limit_point[0] = std::numeric_limits<int>::max();
            limit_point[1] = (std::numeric_limits<int>::max())*(-1);
            limit_point[2] = std::numeric_limits<int>::max();
            limit_point[3] = (std::numeric_limits<int>::max())*(-1);
            get_data();
        }
        void get_data()
        {
            GRIDDING gridding;
            FILE *f;
            f = fopen("dijkstra_lecture_node.csv","r");
            int node;
            float vec_x,vec_y;
            std::vector<float> vec_x_1,vec_y_1,vec_x_0,vec_y_0;
            while(fscanf(f, "%d,%f,%f", &node, &vec_x, &vec_y) != EOF)
            {
                printf("%-1d:  %.3f  %.3f\n", node, vec_x, vec_y);
                vector_2d[zero_point + gridding.float_to_int(vec_x)][zero_point + gridding.float_to_int(vec_y)] = node;
                if ((zero_point + gridding.float_to_int(vec_x)) < limit_point[0])
                {
                    limit_point[0] = zero_point + gridding.float_to_int(vec_x);
                }
                if (limit_point[1] < (zero_point + gridding.float_to_int(vec_x)))
                {
                    limit_point[1] = zero_point + gridding.float_to_int(vec_x);
                }
                if ((zero_point + gridding.float_to_int(vec_y)) < limit_point[2])
                {
                    limit_point[2] = zero_point + gridding.float_to_int(vec_y);
                }
                if (limit_point[3] < (zero_point + gridding.float_to_int(vec_y)))
                {
                    limit_point[3] = zero_point + gridding.float_to_int(vec_y);
                }
                if (node == 1)
                {
                    vec_x_1.push_back(vec_x);
                    vec_y_1.push_back(vec_y);
                }
                else
                {
                    vec_x_0.push_back(vec_x);
                    vec_y_0.push_back(vec_y);
                }
            }
            matplotlibcpp::cla();
            matplotlibcpp::plot(vec_x_1,vec_y_1,"sk");
            matplotlibcpp::plot(vec_x_0,vec_y_0,"sy");
            matplotlibcpp::plot({gridding.float_to_grid(0.0)},{gridding.float_to_grid(0.0)},"or");
            matplotlibcpp::show(2);
            fclose(f);
            std::vector<float> vector_2d_dijkstra_plot_x;
            std::vector<float> vector_2d_dijkstra_plot_y;
            while (ros::ok())
            {
                matplotlibcpp::cla();
                matplotlibcpp::plot(vec_x_1,vec_y_1,"sk");
                matplotlibcpp::plot(vec_x_0,vec_y_0,"sy");
                matplotlibcpp::plot({gridding.float_to_grid(0.0)},{gridding.float_to_grid(0.0)},"or");
                
            }
        }
};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_lecture");
    // std::vector<std::vector<float>> range_point;
    // printf("%f\n",range_point[0][0]);
    // // DIJKSTRA_LECTURE dijkstra_lecture;
    // ros::spin();
    // float angle = tan(M_PI/1.999999);
    float angle = tan(atan2(1., -1.));
    if (std::isnan(angle))
    {
        ROS_INFO("isnan = TRUE");
    }
    if (std::isinf(angle))
    {
        ROS_INFO("isinf = TRUE");
    }
    ROS_INFO("%.2f\n", angle);
    ros::spinOnce();
    ros::spin();
}


// float localization_sita(std::vector<float> me, std::vector<float> fixpose, float fixsita, std::vector<std::vector<float>> ob_global, std::vector<float> obran, std::vector<float> obsita)
// {
//     std::vector<std::vector<float>> ob_local;
//     std::vector<float> oblocal_to_obglobal;
//     float sita_miss = (M_PI/180)*(3);
//     float sita_grid = (M_PI/180)*(0.1);
//     float pose_miss = 2;
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

// std::vector<float> localization_pose(std::vector<float> me, std::vector<float> fixpose, float fixsita, std::vector<std::vector<float>> ob_global, std::vector<std::vector<float>> ob_local)
// {
//     std::vector<float> oblocal_to_obglobal;
//     std::vector<float> new_fixpose{0,0};
//     float pose_miss = 8;
//     float grid = 1;
//     float oblocal_to_obglobal_min;
//     int score, is, js;
//     int score_stack = 0;
//     for (float i=((-1)*pose_miss); i<=pose_miss; i+=grid)
//     {
//         for (float j=((-1)*pose_miss); j<=pose_miss; j+=grid)
//         {
//             score = 0;
//             for (int l=0; l<ob_local.size(); l++)
//             {
//                 for (int m=0; m<ob_global.size(); m++)
//                 {
//                     if ((((ob_global[m][0]-(grid/2)) <= (ob_local[l][0]+i)) && ((ob_local[l][0]+i) < (ob_global[m][0]+(grid/2)))) && (((ob_global[m][1]-(grid/2)) <= (ob_local[l][1]+j)) && ((ob_local[l][1]+j) < (ob_global[m][1]+(grid/2)))))
//                     {
//                         score++;
//                         break;
//                     }
//                 }
//             }
//             if (score_stack < score)
//             {
//                 score_stack = score;
//                 new_fixpose[0] = i;
//                 new_fixpose[1] = j;
//             }
//         }
//     }
//     fixpose[0] += new_fixpose[0];
//     fixpose[1] += new_fixpose[1];
//     return fixpose;
// }






// struct timeval t;   //時間取得用の構造体を定義
// long sec_0, sec;    //時間の差を比較する変数
// ros::spinOnce();
// gettimeofday(&t, NULL);    //時間を取得
// sec_0 = t.tv_sec;     //整数部分の秒(s)
// while (ros::ok())
// {
//     gettimeofday(&t, NULL);    //時間を取得
//     sec = t.tv_sec;     //整数部分の秒(s)
//     ros::spinOnce();
//     pub_start.publish(go);
//     // if (ros::ok()!=true)
//     if ((sec-sec_0) > 3)
//     {
//         break;
//     }
// }