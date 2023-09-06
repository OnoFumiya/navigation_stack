#include <stdio.h>
#include <ros/ros.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
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
// #include <matplotlib-cpp/matplotlibcpp.h>
#include <navigation_stack/MapInformation.h>




/* 
<arg name="save_map"            default="true" />
<node if="$(arg save_map)" name="save_map_command" pkg="sobit_mapping" type="save_map_command.py" launch-prefix="xterm -font r16 -fg floralwhite -bg darkslateblue -e">
    <param name="map_save_path" type="str" value="$(find sobit_mapping)/map/"/>
</node>
*/ 


class MAP_SAVER
{
    private:
        ros::Subscriber sub_map;
        std::string save_file_name_str;
        char save_file_name[100];
        navigation_stack::MapInformation map;
        bool map_set_frag = false;
        YAML::Node yaml_data;
        void callback_map(const navigation_stack::MapInformation &get_map)
        {
            map.cost.clear();
            map.clearly.clear();
            for (int i=0; i<get_map.cost.size(); i++)
            {
                map.cost.push_back(get_map.cost[i]);
            }
            for (int i=0; i<get_map.clearly.size(); i++)
            {
                map.clearly.push_back(get_map.clearly[i]);
            }
            map_set_frag = true;
        }
    public:
        MAP_SAVER()
        {
            ros::NodeHandle node;
            sub_map = node.subscribe("/mapping", 10, &MAP_SAVER::callback_map, this);
            saver();
        }
        void saver()
        {
            printf("\n\nwaiting for map data......\n");
            while (ros::ok())
            {
                if (map_set_frag)
                {
                    break;
                }
                ros::spinOnce();
            }
            while (ros::ok())
            {
                printf("\n\n\n\t\tCommand to save this map\n");
                printf("\nWhen you want to save map, Please enter \"map_name\".\n");
                printf("\tmap_name : ");

                scanf("%s",save_file_name);

                save_file_name_str = std::string(save_file_name);

                if (save_file_name_str == "")
                {
                    // 現在のシステム時刻を取得
                    auto now = std::chrono::system_clock::now();

                    // std::chrono::system_clock::to_time_t を使用して std::time_t に変換
                    std::time_t currentTime = std::chrono::system_clock::to_time_t(now);

                    // std::tm 構造体を使用して詳細な日時情報を取得
                    std::tm* localTime = std::localtime(&currentTime);

                    // 日時情報を格納
                    int year = 1900 + localTime->tm_year;
                    int month = 1 + localTime->tm_mon;
                    int day = localTime->tm_mday;
                    int hour = localTime->tm_hour;
                    int minute = localTime->tm_min;
                    int second = localTime->tm_sec;
                    // std::cout << "Year: " << year << std::endl;
                    // std::cout << "Month: " << month << std::endl;
                    // std::cout << "Day: " << day << std::endl;
                    // std::cout << "Hour: " << hour << std::endl;
                    // std::cout << "Minute: " << minute << std::endl;
                    // std::cout << "Second: " << second << std::endl;
                    save_file_name_str = "map_" + std::to_string(year) + "_" + std::to_string(month) + "_" + std::to_string(day) + "_" + std::to_string(hour) + "_" + std::to_string(minute) + "_" + std::to_string(second);
                }
                save_file_name_str += ".yaml";
                printf("\n\n\nSave this map as %s ...\n\n\n",save_file_name_str.c_str());

                // yaml_data["cost"]
                ros::spinOnce();
                for (int i=0; i<map.cost.size(); i++)
                {
                    yaml_data["map_data"]["cost"][std::to_string(i)]["x"] = map.cost[i].x;
                    yaml_data["map_data"]["cost"][std::to_string(i)]["y"] = map.cost[i].y;
                }
                for (int i=0; i<map.clearly.size(); i++)
                {
                    yaml_data["map_data"]["clearly"][std::to_string(i)]["x"] = map.clearly[i].x;
                    yaml_data["map_data"]["clearly"][std::to_string(i)]["y"] = map.clearly[i].y;
                }
                ros::spinOnce();
                std::string file_path = ros::package::getPath("navigation_stack") + "/map/" + save_file_name_str;
                std::ofstream file(file_path);
                if (file.is_open())
                {
                    file << yaml_data;  // YAMLデータをファイルに書き込む
                    file.close();
                }
                else
                {
                    printf("\x1b[31m\nFailed to open file: %s\n\x1b[0m",file_path.c_str());
                }
                ros::spinOnce();
            }
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_saver");
    MAP_SAVER map_saver;
    ros::spin();
    return 0;
}