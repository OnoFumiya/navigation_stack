#include <rclcpp/rclcpp.hpp>
// #include <dr_spaam_ros/msg/leg_pose_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>
// #include <Eigen/Dense>

using std::placeholders::_1;
using std::vector;

class ALL_PARAMETER {
    public:
        float lidar_pose[2] = {0.2, 0.0};
        int sampling_step;
        float max_human_vel;
        float max_human_radius;
        float human_noise;
        float detect_range_time;
        float velocity_weight_prediction;
        float position_weight_prediction;
        float similar_object;
        float no_moving_range;
        std::string robot_base;
        std::string map;
        std::string removal_scan_topic;
        std::string stepping_point_topic;
        std::string dr_spaam_topic;
        std::string merge_object_topic;
        std::string removal_scan_marker;
        std::string step1_marker;
        std::string step2_marker;
        std::string step3_marker;
        std::string next_step_marker;
        std::string cost_topic;

        ALL_PARAMETER(std::shared_ptr<rclcpp::Node> node)
        {
            node->declare_parameter("sampling_step", 5);
            node->declare_parameter("max_human_vel", 1.5);
            node->declare_parameter("max_human_radius", 0.8);
            node->declare_parameter("human_noise", 0.4);
            node->declare_parameter("detect_range_time", 0.1);
            node->declare_parameter("velocity_weight_prediction", 2.0);
            node->declare_parameter("position_weight_prediction", 1.3333);
            node->declare_parameter("similar_object", 0.2);
            node->declare_parameter("no_moving_range", 0.6);
            node->declare_parameter("robot_base", "base_footprint");
            node->declare_parameter("map", "map");
            node->declare_parameter("removal_scan_topic", "/dr_spaam_navigation/scan");
            node->declare_parameter("stepping_point_topic", "/dr_spaam_navigation/object_pointers_step");
            node->declare_parameter("dr_spaam_topic", "/dr_spaam_detections");
            node->declare_parameter("merge_object_topic", "/object_point");
            node->declare_parameter("removal_scan_marker", "/dr_spaam_navigation/lidar_points");
            node->declare_parameter("step1_marker", "/dr_spaam_navigation/point_step1");
            node->declare_parameter("step2_marker", "/dr_spaam_navigation/point_step2");
            node->declare_parameter("step3_marker", "/dr_spaam_navigation/point_step3");
            node->declare_parameter("next_step_marker", "/dr_spaam_navigation/point_nextstep");
            node->declare_parameter("cost_topic", "/dr_spaam_navigation/point_cost");

            node->get_parameter("sampling_step", sampling_step);
            node->get_parameter("max_human_vel", max_human_vel);
            node->get_parameter("max_human_radius", max_human_radius);
            node->get_parameter("human_noise", human_noise);
            node->get_parameter("detect_range_time", detect_range_time);
            node->get_parameter("velocity_weight_prediction", velocity_weight_prediction);
            node->get_parameter("position_weight_prediction", position_weight_prediction);
            node->get_parameter("similar_object", similar_object);
            node->get_parameter("no_moving_range", no_moving_range);
            node->get_parameter("robot_base", robot_base);
            node->get_parameter("map", map);
            node->get_parameter("removal_scan_topic", removal_scan_topic);
            node->get_parameter("stepping_point_topic", stepping_point_topic);
            node->get_parameter("dr_spaam_topic", dr_spaam_topic);
            node->get_parameter("merge_object_topic", merge_object_topic);
            node->get_parameter("removal_scan_marker", removal_scan_marker);
            node->get_parameter("step1_marker", step1_marker);
            node->get_parameter("step2_marker", step2_marker);
            node->get_parameter("step3_marker", step3_marker);
            node->get_parameter("next_step_marker", next_step_marker);
            node->get_parameter("cost_topic", cost_topic);
        }
};

class MergePointNode : public rclcpp::Node {
    public:
        MergePointNode() : Node("object_point_publisher") {}

        void init() {
            // all_parameter = std::make_unique<ALL_PARAMETER>(shared_from_this());
            all_parameter = std::make_unique<ALL_PARAMETER>(shared_from_this());

            pub_merge_object = this->create_publisher<geometry_msgs::msg::PoseArray>(all_parameter->merge_object_topic, 1);

            leg_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
                all_parameter->dr_spaam_topic, 10, std::bind(&MergePointNode::callback_leg, this, _1));

            scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/sobit_pro/scan", 10, std::bind(&MergePointNode::callback_scan, this, _1));

            timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                std::bind(&MergePointNode::publish_merge_points, this));
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_merge_object;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr leg_sub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        rclcpp::TimerBase::SharedPtr timer_;

        // ALL_PARAMETER all_parameter;
        std::unique_ptr<ALL_PARAMETER> all_parameter;

        geometry_msgs::msg::PoseArray leg_point;
        geometry_msgs::msg::PoseArray merge_data;
        vector<vector<geometry_msgs::msg::Pose>> scan_points;
        vector<geometry_msgs::msg::Pose> g_point;

        bool leg_call_ok = false;
        bool scan_call_ok = false;

        void callback_leg(const geometry_msgs::msg::PoseArray::SharedPtr msg)
        {
            leg_point = *msg;
            leg_call_ok = true;
        }

        void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            scan_points.clear();
            g_point.clear();
            geometry_msgs::msg::Pose pt;
            pt.position.z = 0.2;
            pt.orientation.w = 1.0;

            for (size_t i = 0; i < msg->ranges.size(); ++i)
            {
                if (!std::isnan(msg->ranges[i]) && msg->ranges[i] > 0.1)
                {
                    pt.position.x = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i);
                    pt.position.y = msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i);

                    bool inputer = false;
                    for (size_t j = 0; j < scan_points.size(); ++j)
                    {
                        const auto& last = scan_points[j].back();
                        float dist = std::hypot(last.position.x - pt.position.x, last.position.y - pt.position.y);
                        if (dist < all_parameter->similar_object)
                        {
                            g_point[j].position.x = (g_point[j].position.x * scan_points[j].size() + pt.position.x) / (scan_points[j].size() + 1);
                            g_point[j].position.y = (g_point[j].position.y * scan_points[j].size() + pt.position.y) / (scan_points[j].size() + 1);
                            scan_points[j].push_back(pt);
                            inputer = true;
                            break;
                        }
                    }
                    if (!inputer)
                    {
                        scan_points.push_back({pt});
                        g_point.push_back(pt);
                    }
                }
            }

            for (int i = static_cast<int>(scan_points.size()) - 1; i >= 0; --i)
            {
                const auto& points = scan_points[i];
                float dist = std::hypot(points.back().position.x - points.front().position.x,
                                        points.back().position.y - points.front().position.y);
                if (dist > all_parameter->no_moving_range)
                {
                    g_point.erase(g_point.begin() + i);
                }
            }

            scan_call_ok = true;
        }

        void publish_merge_points()
        {
            if (!(leg_call_ok && scan_call_ok)) return;

            merge_data = leg_point;
            merge_data.poses = g_point;

            for (int i = static_cast<int>(leg_point.poses.size()) - 1; i >= 0; --i)
            {
                for (int j = static_cast<int>(merge_data.poses.size()) - 1; j >= 0; --j)
                {
                    float dist = std::hypot(
                        leg_point.poses[i].position.x - merge_data.poses[j].position.x,
                        leg_point.poses[i].position.y - merge_data.poses[j].position.y);

                    if (dist <= all_parameter->human_noise)
                    {
                        merge_data.poses.erase(merge_data.poses.begin() + j);
                    }
                }
                merge_data.poses.push_back(leg_point.poses[i]);
            }

            if (merge_data.poses.empty())
            {
                geometry_msgs::msg::Pose no_pose;
                no_pose.position.x = 100.0;
                no_pose.position.y = 100.0;
                no_pose.position.z = 0.0;
                no_pose.orientation.w = 1.0;
                merge_data.poses.push_back(no_pose);
            }

            pub_merge_object->publish(merge_data);
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MergePointNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
