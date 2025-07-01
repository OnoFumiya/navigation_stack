#include <rclcpp/rclcpp.hpp>
// #include <dr_spaam_ros/msg/leg_pose_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
// #include <navigation_stack/msg/walk_leg_point.hpp>
// #include <std_srvs/srv/empty.hpp>
#include <nav2_msgs/srv/clear_costmap_except_region.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <algorithm>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>


#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
// using DrspaamScanSyncPolicy = message_filters::sync_policies::ApproximateTime<
//   geometry_msgs::msg::PoseArray,
//   sensor_msgs::msg::LaserScan>;
// typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseArray, sensor_msgs::msg::LaserScan> DrspaamScanSyncPolicy;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


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
        int clear_per_times;
        float trajectry_weight[5];
        std::string robot_base;
        std::string map;
        std::string removal_scan_topic;
        // std::string stepping_point_topic;
        std::string dr_spaam_topic;
        std::string merge_object_topic;
        std::string removal_scan_marker;
        std::string step1_marker;
        std::string step2_marker;
        std::string step3_marker;
        std::string next_step_marker;
        std::string cost_topic;

        ALL_PARAMETER(std::shared_ptr<rclcpp::Node> node) {
            node->declare_parameter("sampling_step", 5);
            node->declare_parameter("max_human_vel", 1.5);
            node->declare_parameter("max_human_radius", 0.8);
            node->declare_parameter("human_noise", 0.4);
            node->declare_parameter("detect_range_time", 0.1);
            node->declare_parameter("velocity_weight_prediction", 2.0);
            node->declare_parameter("position_weight_prediction", 1.3333);
            node->declare_parameter("similar_object", 0.2);
            node->declare_parameter("no_moving_range", 0.6);
            node->declare_parameter("clear_per_times", 1);
            node->declare_parameter("trajectry_weight.line", 0.1);
            node->declare_parameter("trajectry_weight.parabola", 0.1);
            node->declare_parameter("trajectry_weight.y_parabola", 0.1);
            node->declare_parameter("trajectry_weight.sigmoid", 0.1);
            node->declare_parameter("trajectry_weight.y_sigmoid", 0.1);
            node->declare_parameter("robot_base", "base_footprint");
            node->declare_parameter("map", "map");
            node->declare_parameter("removal_scan_topic", "/dr_spaam_navigation/scan");
            // node->declare_parameter("stepping_point_topic", "/dr_spaam_navigation/object_pointers_step");
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
            // node->get_parameter("stepping_point_topic", stepping_point_topic);
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

class ObjectDetectNode : public rclcpp::Node {
    public:
        ObjectDetectNode() : Node("step_point_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {}

        void init() {
            all_parameter = std::make_unique<ALL_PARAMETER>(shared_from_this());

            robot_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/sobit_pro/cmd_vel", 10, std::bind(&ObjectDetectNode::callback_robotvel, this, _1));

            pub_point1_ = create_publisher<geometry_msgs::msg::PoseArray>("/dr_spaam_navigation/object_pointers_1", 1);
            pub_point2_ = create_publisher<geometry_msgs::msg::PoseArray>("/dr_spaam_navigation/object_pointers_2", 1);
            pub_point3_ = create_publisher<geometry_msgs::msg::PoseArray>("/dr_spaam_navigation/object_pointers_3", 1);
            pub_next_point_ = create_publisher<geometry_msgs::msg::PoseArray>("/dr_spaam_navigation/object_next_pointers", 1);
            pub_removed_scan_ = create_publisher<sensor_msgs::msg::LaserScan>(all_parameter->removal_scan_topic, 10);
            pub_object_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(all_parameter->cost_topic, 1);

            clear_client_ = this->create_client<nav2_msgs::srv::ClearCostmapExceptRegion>("/local_costmap/clear_except_local_costmap");
            // clear_client_->wait_for_service(1s)
            while (!clear_client_->wait_for_service(1s)) {}

            // sub_legs_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseArray>>(this, all_parameter->merge_object_topic);
            // sub_scan_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, "/sobit_pro/scan");
            // sync_     = std::make_shared<message_filters::Synchronizer<DrspaamScanSyncPolicy>>(DrspaamScanSyncPolicy(500), *sub_legs_, *sub_scan_);
            // sync_->registerCallback(&ObjectDetectNode::callback_object, this);

            sub_legs_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
                all_parameter->merge_object_topic, 1, std::bind(&ObjectDetectNode::callback_object, this, _1));
            sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/sobit_pro/scan", 1, std::bind(&ObjectDetectNode::callback, this, _1));

            start_flag_ = false;
            scan_flag_ = false;
            rclcpp::Rate r(5);
            while (rclcpp::ok()) {
                r.sleep();
                // rclcpp::spin_once(this, timeout_sec=0.2);
                rclcpp::spin_some(shared_from_this());
                if (start_flag_) break;
            }
            point1_.poses.resize(object_points_.poses.size());
            point2_.poses.resize(object_points_.poses.size());
            point3_.poses.resize(object_points_.poses.size());
            copy(object_points_.poses.begin(), object_points_.poses.end(), point1_.poses.begin());
            copy(object_points_.poses.begin(), object_points_.poses.end(), point2_.poses.begin());
            copy(object_points_.poses.begin(), object_points_.poses.end(), point3_.poses.begin());
            point_next_.poses.clear();
            counter_ = 0;

            RCLCPP_INFO(this->get_logger(), "Waiting first synced message...");

            // timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1/(all_parameter->detect_range_time*all_parameter->sampling_step))),
            //     std::bind(&ObjectDetectNode::publish_step_points, this));

            rclcpp::WallRate loop(1/all_parameter->detect_range_time);
            while (rclcpp::ok()) {
                loop.sleep();
                // sleep(all_parameter->detect_range_time);
                // sleep(0.5);
                publish_step_points();
                rclcpp::spin_some(shared_from_this());
            }
        }

    private:
        void callback_robotvel(const geometry_msgs::msg::Twist::SharedPtr msg) {
            robot_vel_ = *msg;
        }

        void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "SCAN");
            scan_ = *msg;
            header_ = msg->header;
            scan_flag_ = true;
        }

        geometry_msgs::msg::Point transform_point(
        const std::string &from, const std::string &to,
        const geometry_msgs::msg::Point &pt)
        {
            geometry_msgs::msg::PointStamped in, out;
            in.header.frame_id = from;
            in.header.stamp.sec = 0;
            in.header.stamp.nanosec = 0;
            in.point = pt;
            try {
                tf_buffer_.transform(in, out, to);
                return out.point;
            } catch (tf2::TransformException &e) {
                RCLCPP_WARN(get_logger(), "TF error: %s", e.what());
                return geometry_msgs::msg::Point();
            }
        }

        void callback_object(
            const geometry_msgs::msg::PoseArray::SharedPtr msg)
        {
            if (!scan_flag_) return;
            points_base_.poses.clear();
            object_points_.poses.clear();
            for (auto &pose : msg->poses) {
                geometry_msgs::msg::Pose pt;
                pt.position.x = pose.position.x + all_parameter->lidar_pose[0];
                pt.position.y = pose.position.y + all_parameter->lidar_pose[1];
                pt.position.z = 0.2;
                double r = std::hypot(pt.position.x, pt.position.y);
                if (r >= 0.25 && r <= scan_.range_max) {
                    points_base_.poses.push_back(pt);
                    geometry_msgs::msg::Pose pos;
                    pos.position = transform_point(all_parameter->robot_base, all_parameter->map, pt.position);
                    object_points_.poses.push_back(pos);
                }
            }

            auto out = scan_;
            for (size_t i = 0; i < scan_.ranges.size(); ++i) {
                double x = scan_.ranges[i]
                * std::cos(scan_.angle_min + i * scan_.angle_increment)
                + all_parameter->lidar_pose[0];
                double y = scan_.ranges[i]
                * std::sin(scan_.angle_min + i * scan_.angle_increment)
                + all_parameter->lidar_pose[1];
                for (auto &ob : points_base_.poses) {
                if (std::hypot(x - ob.position.x, y - ob.position.y) <= all_parameter->max_human_radius) {
                    out.ranges[i] = std::numeric_limits<double>::quiet_NaN();
                    break;
                }
                }
            }

            pub_removed_scan_->publish(out);
            start_flag_ = true;
        }


        // 近似直線
        void line_path(const std::vector<geometry_msgs::msg::Point> pts, 
                       double &a, double &b, bool &flag, 
                       std::vector<geometry_msgs::msg::Point> &s_pts) {
            // 例外
            flag = true;
            if ((3*(std::pow(pts[0].x, 2) + std::pow(pts[1].x, 2) + std::pow(pts[2].x, 2))) == std::pow(pts[0].x + pts[1].x + pts[2].x, 2)) flag = false;

            // 算出
            if (flag) {
                double sum_x, sum_y, sum_xx, sum_xy;
                sum_x = pts[0].x + pts[1].x + pts[2].x;
                sum_y = pts[0].y + pts[1].y + pts[2].y;
                sum_xx = std::pow(pts[0].x, 2) + std::pow(pts[1].x, 2) + std::pow(pts[2].x, 2);
                sum_xy = pts[0].x*pts[0].y + pts[1].x*pts[1].y + pts[2].x*pts[2].y;
                a = (3.0 * sum_xy - sum_x * sum_y) / (3.0 * sum_xx - sum_x * sum_x);
                b = (sum_y - a * sum_x) / 3.0;
            }

            // 棄却許容判定
            if (flag) {
                // double samp_x1, samp_x2, samp_x3, samp_y1, samp_y2, samp_y3;
                s_pts[0].x = (-a*b + a*pts[0].y + pts[0].x) / (std::pow(a, 2.) + 1.);
                s_pts[0].y = (-a*b + a*pts[0].y + pts[0].x) / (std::pow(a, 2.) + 1.) + b;
                s_pts[1].x = (-a*b + a*pts[1].y + pts[1].x) / (std::pow(a, 2.) + 1.);
                s_pts[1].y = (-a*b + a*pts[1].y + pts[1].x) / (std::pow(a, 2.) + 1.) + b;
                s_pts[2].x = (-a*b + a*pts[2].y + pts[2].x) / (std::pow(a, 2.) + 1.);
                s_pts[2].y = (-a*b + a*pts[2].y + pts[2].x) / (std::pow(a, 2.) + 1.) + b;
                if (((((s_pts[0].x <= s_pts[1].x) && (s_pts[1].x <= s_pts[2].x)) || ((s_pts[2].x <= s_pts[1].x) && (s_pts[1].x <= s_pts[0].x))) && (s_pts[2].x != s_pts[0].x)) ||
                    ((((s_pts[0].y <= s_pts[1].y) && (s_pts[1].y <= s_pts[2].y)) || ((s_pts[2].y <= s_pts[1].y) && (s_pts[1].y <= s_pts[0].y))) && (s_pts[2].y != s_pts[0].y))) {
                        flag = true;
                } else  flag = false;
            }
        }


        // 放物線
        void parabola(const std::vector<geometry_msgs::msg::Point> pts, 
                      double &ax, double &bx, double &cx, bool &flag_x, double &ay, double &by, double &cy, bool &flag_y) {
            // 例外
            flag_x = true;
            flag_y = true;
            if ((pts[0].x == pts[1].x) || (pts[1].x == pts[2].x) || (pts[2].x == pts[0].x)) flag_x = false;
            if ((pts[0].y == pts[1].y) || (pts[1].y == pts[2].y) || (pts[2].y == pts[0].y)) flag_y = false;

            // 算出
            if (flag_x) {
                ax = (pts[0].x*(pts[2].y - pts[1].y) + pts[1].x*(pts[0].y - pts[2].y) + pts[2].x*(pts[1].y - pts[0].y)) / ((pts[0].x - pts[1].x) * (pts[0].x - pts[2].x) * (pts[1].x - pts[2].x));
                bx = ((std::pow(pts[1].x, 2) - std::pow(pts[2].x, 2))*pts[0].y + (std::pow(pts[2].x, 2) - std::pow(pts[0].x, 2))*pts[1].y + (std::pow(pts[0].x, 2) - std::pow(pts[1].x, 2))*pts[2].y) / ((pts[0].x - pts[1].x) * (pts[0].x - pts[2].x) * (pts[2].x - pts[1].x));
                cx = pts[0].y - ax*std::pow(pts[0].x, 2) -bx*pts[0].x;
            }
            if (flag_y) {
                ay= (pts[0].y*(pts[2].x - pts[1].x) + pts[1].y*(pts[0].x - pts[2].x) + pts[2].y*(pts[1].x - pts[0].x)) / ((pts[0].y - pts[1].y) * (pts[0].y - pts[2].y) * (pts[1].y - pts[2].y));
                by= ((std::pow(pts[1].y, 2) - std::pow(pts[2].y, 2))*pts[0].x + (std::pow(pts[2].y, 2) - std::pow(pts[0].y, 2))*pts[1].x + (std::pow(pts[0].y, 2) - std::pow(pts[1].y, 2))*pts[2].x) / ((pts[0].y - pts[1].y) * (pts[0].y - pts[2].y) * (pts[2].y - pts[1].y));
                cy= pts[0].x - ay*std::pow(pts[0].y, 2) -by*pts[0].y;
            }

            // 棄却許容判定
            if (flag_x) {
                if ((((pts[0].x <= pts[1].x) && (pts[1].x <= pts[2].x)) || ((pts[2].x <= pts[1].x) && (pts[1].x <= pts[0].x))) && (pts[2].x != pts[0].x)) {
                        flag_x = true;
                } else  flag_x = false;
                double pre_x, pre_y;
                pre_x = ( ((std::pow(2*ax*pts[2].x+bx,2) + 1)*pts[2].x) + ((pts[2].x-pts[1].x)/std::fabs(pts[2].x-pts[1].x)) * std::sqrt(std::pow((std::pow(2*ax*pts[2].x+bx,2) + 1)*pts[2].x,2) - (std::pow(2*ax*pts[2].x+bx,2) + 1)*(std::pow(pts[2].x,2)*(std::pow(2*ax*pts[2].x+bx,2) + 1)-1)) ) / ( std::pow(2*ax*pts[2].x+bx,2) + 1 );
                pre_y = (2*ax*pts[2].x + bx)*pre_x + pts[2].y - (2*ax*pts[2].x + bx) * pts[2].x;
                if ((((pts[0].x < pts[2].x) && (pts[2].x < -(bx)/(2*ax))) || ((-(bx)/(2*ax) < pts[2].x) && (pts[2].x < pts[0].x))) || 
                    ( acos(   ( (pts[2].x-pts[1].x) * (pre_x-pts[2].x) + (pts[2].y-pts[1].y) * (pre_y-pts[2].y) ) / std::sqrt(std::pow(pts[2].x-pts[1].x, 2) + std::pow(pts[2].y-pts[1].y, 2))   ) > M_PI/2. )) {
                    flag_x = false;
                }
            }
            if (flag_y) {
                if ((((pts[0].y <= pts[1].y) && (pts[1].y <= pts[2].y)) || ((pts[2].y <= pts[1].y) && (pts[1].y <= pts[0].y))) && (pts[2].y != pts[0].y)) {
                        flag_y = true;
                } else  flag_y = false;
                double pre_x, pre_y;
                pre_y = ( ((std::pow(2*ay*pts[2].y+by,2) + 1)*pts[2].y) + ((pts[2].y-pts[1].y)/std::fabs(pts[2].y-pts[1].y)) * std::sqrt(std::pow((std::pow(2*ay*pts[2].y+by,2) + 1)*pts[2].y,2) - (std::pow(2*ay*pts[2].y+by,2) + 1)*(std::pow(pts[2].y,2)*(std::pow(2*ay*pts[2].y+by,2) + 1)-1)) ) / ( std::pow(2*ay*pts[2].y+by,2) + 1 );
                pre_x = (2*ay*pts[2].y + by)*pre_y + pts[2].x - (2*ay*pts[2].y + by) * pts[2].y;
                if ((((pts[0].y < pts[2].y) && (pts[2].y < -(by)/(2*ay))) || ((-(by)/(2*ay) < pts[2].y) && (pts[2].y < pts[0].y))) || 
                    ( acos(   ( (pts[2].y-pts[1].y) * (pre_y-pts[2].y) + (pts[2].x-pts[1].x) * (pre_x-pts[2].x) ) / std::sqrt(std::pow(pts[2].y-pts[1].y, 2) + std::pow(pts[2].x-pts[1].x, 2))   ) > M_PI/2. )) {
                    flag_y = false;
                }
            }
        }


        // シグモイド曲線
        void sigmoid(const std::vector<geometry_msgs::msg::Point> pts, 
                     double &Lx, double &ax, double &dxx, double &dyx, bool &flag_x, double &Ly, double &ay, double &dxy, double &dyy, bool &flag_y) {
            // 例外
            flag_x = true;
            flag_y = true;
            if ((pts[2].y == pts[1].y) || (pts[2].x == pts[1].x)) {
                flag_x = false;
                flag_y = false;
            }

            // 算出
            if (flag_x && flag_y) {
                dxx = pts[1].x;
                Lx = 2 * std::fabs(pts[2].y-pts[1].y);
                dyx = pts[1].y - (Lx / 2.0);
                ax = 4 * ((pts[2].y - pts[1].y) / (pts[2].x - pts[1].x)) / Lx;

                dyy = pts[1].y;
                Ly = 2 * std::fabs(pts[2].x-pts[1].x);
                dxy = pts[1].x - (Ly / 2.0);
                ay = 4 * ((pts[2].x - pts[1].x) / (pts[2].y - pts[1].y)) / Ly;
            }

            // 棄却許容判定
            if (flag_x && flag_y) {
                if ((((pts[0].x <= pts[1].x) && (pts[1].x <= pts[2].x)) || ((pts[2].x <= pts[1].x) && (pts[1].x <= pts[0].x))) && (pts[2].x != pts[0].x)) {
                        flag_x = true;
                } else  flag_x = false;
                if ((((pts[0].y <= pts[1].y) && (pts[1].y <= pts[2].y)) || ((pts[2].y <= pts[1].y) && (pts[1].y <= pts[0].y))) && (pts[2].y != pts[0].y)) {
                        flag_y = true;
                } else  flag_y = false;
                if (flag_x) {
                    if ( (( atan((pts[2].y-pts[0].y) / (pts[2].x-pts[0].x))/2. + atan((pts[2].y-pts[1].y) / (pts[2].x-pts[1].x))/2. ) < (-5*M_PI/6.)) && ((5*M_PI/6.) < ( atan((pts[2].y-pts[0].y) / (pts[2].x-pts[0].x))/2. + atan((pts[2].y-pts[1].y) / (pts[2].x-pts[1].x))/2. )) ) flag_x = false;
                }
                if (flag_y) {
                    if ( (-M_PI/6.) < (( atan((pts[2].y-pts[0].y) / (pts[2].x-pts[0].x))/2. + atan((pts[2].y-pts[1].y) / (pts[2].x-pts[1].x))/2. )) && (( atan((pts[2].y-pts[0].y) / (pts[2].x-pts[0].x))/2. + atan((pts[2].y-pts[1].y) / (pts[2].x-pts[1].x))/2. ) < (M_PI/6.)) ) flag_y = false;
                }
            }
        }

        void publish_step_points() {
            if (!start_flag_) return;

            PointCloud::Ptr pcl_cloud(new PointCloud());
            sensor_msgs::msg::PointCloud2 cloud_msg;

            point1_.header = header_;
            point2_.header = header_;
            point3_.header = header_;
            point_next_.header = header_;

            // 新たなステップへ更新
            point1_.poses.clear();
            point1_.poses.resize(point2_.poses.size());
            copy(point2_.poses.begin(), point2_.poses.end(), point1_.poses.begin());
            point2_.poses.clear();
            point2_.poses.resize(point3_.poses.size());
            copy(point3_.poses.begin(), point3_.poses.end(), point2_.poses.begin());

            std::vector<std::vector<geometry_msgs::msg::Point>> object_points_samplings;   // 第一要素が物体の数、第二要素が物体のそれぞれのステップ
            geometry_msgs::msg::PoseArray object_points_temp;
            object_points_samplings.clear();
            object_points_samplings.resize(object_points_.poses.size());
            object_points_temp.poses.resize(object_points_.poses.size());
            copy(object_points_.poses.begin(), object_points_.poses.end(), object_points_temp.poses.begin());
            for (size_t i=0; i<object_points_temp.poses.size(); i++) {
                object_points_samplings[i].push_back(object_points_temp.poses[i].position);
            }

            rclcpp::WallRate loop(1/all_parameter->detect_range_time);
            for (int i=1; i<=all_parameter->sampling_step; i++) {
                loop.sleep();
                // sleep(all_parameter->detect_range_time);
                rclcpp::spin_some(shared_from_this());
                object_points_temp.poses.resize(object_points_.poses.size());
                copy(object_points_.poses.begin(), object_points_.poses.end(), object_points_temp.poses.begin());
                for (size_t j=0; j<object_points_temp.poses.size(); j++) {
                    bool input_flag = false;
                    for (size_t k=0; k<object_points_samplings.size(); k++) {
                        if (std::sqrt(std::pow(object_points_temp.poses[j].position.x - object_points_samplings[k][object_points_samplings[k].size()-1].x, 2.) + std::pow(object_points_temp.poses[j].position.y - object_points_samplings[k][object_points_samplings[k].size()-1].y, 2.)) <= all_parameter->human_noise) {
                            object_points_samplings[k].push_back(object_points_temp.poses[j].position);
                            input_flag = true;
                        }
                    }
                    if (input_flag != true) {
                        std::vector<geometry_msgs::msg::Point> new_input_object;
                        new_input_object.clear();
                        new_input_object.push_back(object_points_temp.poses[j].position);
                        object_points_samplings.push_back(new_input_object);
                    }
                }
            }
            if (0 < all_parameter->clear_per_times) {
                if (all_parameter->clear_per_times <= counter_) {
                    // clear_client_.call(emp_srv_);  /// TODO
                    auto emp_srv_ = std::make_shared<nav2_msgs::srv::ClearCostmapExceptRegion::Request>();
                    emp_srv_->reset_distance = 0.;
                    auto result = clear_client_->async_send_request(emp_srv_);
                    rclcpp::spin_until_future_complete(shared_from_this(), result);
                    counter_ = 1;
                }
                else counter_++;
            }

            geometry_msgs::msg::Pose ob_point_temp;
            geometry_msgs::msg::Pose pt_nan;
            pt_nan.position.x = NAN;
            pt_nan.position.y = NAN;
            pt_nan.position.z = 0.0;
            point3_.poses.clear();
            point3_.poses.resize(point2_.poses.size(), pt_nan);

            for (size_t i=0; i<object_points_samplings.size(); i++) {
                float x = 0.0;
                float y = 0.0;
                for (size_t j=0; j<object_points_samplings[i].size(); j++) {
                    x += object_points_samplings[i][j].x;
                    y += object_points_samplings[i][j].y;
                }
                ob_point_temp.position.x = x/object_points_samplings[i].size();
                ob_point_temp.position.y = y/object_points_samplings[i].size();
                ob_point_temp.position.z = 0.2;
                bool input_flag = false;
                for (size_t j=0; j<point2_.poses.size(); j++) {
                    if (sqrtf(powf(ob_point_temp.position.x - point2_.poses[j].position.x, 2.) + powf(ob_point_temp.position.y - point2_.poses[j].position.y, 2.)) <= all_parameter->max_human_vel*all_parameter->detect_range_time*all_parameter->sampling_step) {
                        point3_.poses[j].position.x = ob_point_temp.position.x;
                        point3_.poses[j].position.y = ob_point_temp.position.y;
                        point3_.poses[j].position.z = ob_point_temp.position.z;
                        input_flag = true;
                        break;
                    }
                }
                if (!input_flag) point3_.poses.push_back(ob_point_temp);
            }

            for (size_t i=0; i<point3_.poses.size(); i++) {
                if ((std::isnan(point3_.poses[i].position.x)) || (std::isnan(point3_.poses[i].position.y))) {
                    point3_.poses.erase(point3_.poses.begin() + i);
                    if (i < point1_.poses.size()) point1_.poses.erase(point1_.poses.begin() + i);
                    if (i < point2_.poses.size()) point2_.poses.erase(point2_.poses.begin() + i);
                }
            }
            // 新たなステップへ更新 //


            // 軌道予測
            geometry_msgs::msg::Point p1, p2, p3;
            point_next_.poses.clear();

            for (size_t i=0; i<point3_.poses.size(); i++) {
                // 例外処理
                if ((point1_.poses.size() <= i) || (point2_.poses.size() <= i)) break;
                if ((std::isnan(point1_.poses[i].position.x)) || (std::isnan(point1_.poses[i].position.y)) || (std::isnan(point2_.poses[i].position.x)) || (std::isnan(point2_.poses[i].position.y)) || (std::isnan(point3_.poses[i].position.x)) || (std::isnan(point3_.poses[i].position.y))) continue;
            
                p1 = transform_point(all_parameter->map, all_parameter->robot_base, point1_.poses[i].position);
                p2 = transform_point(all_parameter->map, all_parameter->robot_base, point2_.poses[i].position);
                p3 = transform_point(all_parameter->map, all_parameter->robot_base, point3_.poses[i].position);

                if ((std::sqrt(std::pow(p3.x - p2.x, 2) + std::pow(p3.y - p2.y, 2))/(2*all_parameter->detect_range_time) < all_parameter->human_noise) || (all_parameter->max_human_vel < std::sqrt(std::pow(p3.x - p2.x, 2) + std::pow(p3.y - p2.y, 2))/(2*all_parameter->detect_range_time))) {
                    geometry_msgs::msg::Pose pos;
                    pos.position = p3;
                    PointT p;
                    p.x = p3.x;
                    p.y = p3.y;
                    p.z = p3.z;
                    pcl_cloud->points.push_back(p);
                    point_next_.poses.push_back(pos);
                    continue;
                }

                // 3つの軌道予測（type1:近似直線，type2:x軸が変数の放物線，type3:y軸が変数の放物線，type4:x軸が変数のシグモイド曲線，type5:y軸が変数のシグモイド曲線）
                std::vector<geometry_msgs::msg::Point> pts_s_1d({p1, p2, p3});
                double a_1d, b_1b;
                double a_2d, b_2d, c_2d;
                double ta_2d, tb_2d, tc_2d;
                double L_sig, a_sig, dx_sig, dy_sig;
                double tL_sig, ta_sig, tdx_sig, tdy_sig;
                bool check_type1, check_type2, check_type3, check_type4, check_type5;
                line_path({p1, p2, p3}, a_1d, b_1b, check_type1, pts_s_1d);
                parabola({p1, p2, p3}, a_2d, b_2d, c_2d, check_type2, ta_2d, tb_2d, tc_2d, check_type3);
                sigmoid({p1, p2, p3}, L_sig, a_sig, dx_sig, dy_sig, check_type4, tL_sig, ta_sig, tdx_sig, tdy_sig, check_type5);

                double sum_weight = 0.;
                if (check_type1) sum_weight += all_parameter->trajectry_weight[0];
                if (check_type2) sum_weight += all_parameter->trajectry_weight[1];
                if (check_type3) sum_weight += all_parameter->trajectry_weight[2];
                if (check_type4) sum_weight += all_parameter->trajectry_weight[3];
                if (check_type5) sum_weight += all_parameter->trajectry_weight[4];
                if (sum_weight == 0.) continue;

                double dist_robot_ob = std::sqrt(std::pow(p3.x-0., 2.) + std::pow(p3.y-0., 2.));
                geometry_msgs::msg::Point next_pt;
                next_pt.x = p3.x;
                next_pt.y = p3.y;
                next_pt.z = 0.3;

                int step;
                for (step=1; ; step++) {
                    geometry_msgs::msg::Point robot_pt;
                    robot_pt.x = robot_vel_.linear.x * all_parameter->detect_range_time * (step + 1) + all_parameter->lidar_pose[0];
                    robot_pt.y = robot_vel_.linear.y * all_parameter->detect_range_time * (step + 1) + all_parameter->lidar_pose[1];
                    robot_pt.z = 0.;

                    geometry_msgs::msg::Point ob_pt_type1, ob_pt_type2, ob_pt_type3, ob_pt_type4, ob_pt_type5, sampling_pt;
                    if (check_type1) {
                        ob_pt_type1.x = p3.x + (pts_s_1d[2].x - pts_s_1d[0].x) / std::fabs(pts_s_1d[2].x - pts_s_1d[0].x) * all_parameter->detect_range_time * step;
                        ob_pt_type1.y = a_1d * ob_pt_type1.x + b_1b;
                        ob_pt_type1.z = 0.;
                    }
                    if (check_type2) {
                        ob_pt_type2.x = p3.x + (p3.x - p1.x) / std::fabs(p3.x - p1.x) * all_parameter->detect_range_time * step;
                        ob_pt_type2.y = a_2d * std::pow(ob_pt_type2.x, 2.) + b_2d * ob_pt_type2.x + c_2d;
                        ob_pt_type2.z = 0.;
                    }
                    if (check_type3) {
                        ob_pt_type3.y = p3.y + (p3.y - p1.y) / std::fabs(p3.y - p1.y) * all_parameter->detect_range_time * step;
                        ob_pt_type3.x = ta_2d * std::pow(ob_pt_type3.y, 2.) + tb_2d * ob_pt_type3.y + tc_2d;
                        ob_pt_type3.z = 0.;
                    }
                    if (check_type4) {
                        ob_pt_type4.x = p3.x + (p3.x - p2.x) / std::fabs(p3.x - p2.x) * all_parameter->detect_range_time * step;
                        ob_pt_type4.y = L_sig / (1 + exp(-a_sig * (ob_pt_type4.x - dx_sig))) + dy_sig;
                        ob_pt_type4.z = 0.;
                    }
                    if (check_type5) {
                        ob_pt_type5.y = p3.y + (p3.y - p2.y) / std::fabs(p3.y - p2.y) * all_parameter->detect_range_time * step;
                        ob_pt_type5.x = tL_sig / (1 + exp(-ta_sig * (ob_pt_type5.y - tdy_sig))) + tdx_sig;
                        ob_pt_type5.z = 0.;
                    }

                    sampling_pt.x = 0.;
                    sampling_pt.y = 0.;
                    sampling_pt.z = 0.;
                    if (check_type1) {
                        sampling_pt.x += ob_pt_type1.x * (all_parameter->trajectry_weight[0] / sum_weight);
                        sampling_pt.y += ob_pt_type1.y * (all_parameter->trajectry_weight[0] / sum_weight);
                    }
                    if (check_type2) {
                        sampling_pt.x += ob_pt_type2.x * (all_parameter->trajectry_weight[1] / sum_weight);
                        sampling_pt.y += ob_pt_type2.y * (all_parameter->trajectry_weight[1] / sum_weight);
                    }
                    if (check_type3) {
                        sampling_pt.x += ob_pt_type3.x * (all_parameter->trajectry_weight[2] / sum_weight);
                        sampling_pt.y += ob_pt_type3.y * (all_parameter->trajectry_weight[2] / sum_weight);
                    }
                    if (check_type4) {
                        sampling_pt.x += ob_pt_type4.x * (all_parameter->trajectry_weight[3] / sum_weight);
                        sampling_pt.y += ob_pt_type4.y * (all_parameter->trajectry_weight[3] / sum_weight);
                    }
                    if (check_type5) {
                        sampling_pt.x += ob_pt_type5.x * (all_parameter->trajectry_weight[4] / sum_weight);
                        sampling_pt.y += ob_pt_type5.y * (all_parameter->trajectry_weight[4] / sum_weight);
                    }
                    if (std::sqrt(std::pow(sampling_pt.x - robot_pt.x, 2.) + std::pow(sampling_pt.y - robot_pt.y, 2.)) < dist_robot_ob) {
                        dist_robot_ob = std::sqrt(std::pow(sampling_pt.x - robot_pt.x, 2.) + std::pow(sampling_pt.y - robot_pt.y, 2.));
                        next_pt.x = sampling_pt.x;
                        next_pt.y = sampling_pt.y;
                    }
                    else break;
                }

                if (step == 1) {
                    geometry_msgs::msg::Point robot_pt;
                    robot_pt.x = -robot_vel_.linear.x * all_parameter->detect_range_time + all_parameter->lidar_pose[0];
                    robot_pt.y = -robot_vel_.linear.y * all_parameter->detect_range_time + all_parameter->lidar_pose[1];
                    robot_pt.z = 0.;

                    geometry_msgs::msg::Point ob_pt_type1, ob_pt_type2, ob_pt_type3, ob_pt_type4, ob_pt_type5, sampling_pt;
                    if (check_type1) {
                        ob_pt_type1.x = p3.x - (pts_s_1d[2].x - pts_s_1d[0].x) / std::fabs(pts_s_1d[2].x - pts_s_1d[0].x) * all_parameter->detect_range_time;
                        ob_pt_type1.y = a_1d * ob_pt_type1.x + b_1b;
                        ob_pt_type1.z = 0.;
                    }
                    if (check_type2) {
                        ob_pt_type2.x = p3.x - (p3.x - p1.x) / std::fabs(p3.x - p1.x) * all_parameter->detect_range_time;
                        ob_pt_type2.y = a_2d * std::pow(ob_pt_type2.x, 2.) + b_2d * ob_pt_type2.x + c_2d;
                        ob_pt_type2.z = 0.;
                    }
                    if (check_type3) {
                        ob_pt_type3.y = p3.y - (p3.y - p1.y) / std::fabs(p3.y - p1.y) * all_parameter->detect_range_time;
                        ob_pt_type3.x = ta_2d * std::pow(ob_pt_type3.y, 2.) + tb_2d * ob_pt_type3.y + tc_2d;
                        ob_pt_type3.z = 0.;
                    }
                    if (check_type4) {
                        ob_pt_type4.x = p3.x - (p3.x - p2.x) / std::fabs(p3.x - p2.x) * all_parameter->detect_range_time;
                        ob_pt_type4.y = L_sig / (1 + exp(-a_sig * (ob_pt_type4.x - dx_sig))) + dy_sig;
                        ob_pt_type4.z = 0.;
                    }
                    if (check_type5) {
                        ob_pt_type5.y = p3.y - (p3.y - p2.y) / std::fabs(p3.y - p2.y) * all_parameter->detect_range_time;
                        ob_pt_type5.x = tL_sig / (1 + exp(-ta_sig * (ob_pt_type5.y - tdy_sig))) + tdx_sig;
                        ob_pt_type5.z = 0.;
                    }

                    sampling_pt.x = 0.;
                    sampling_pt.y = 0.;
                    sampling_pt.z = 0.;
                    if (check_type1) {
                        sampling_pt.x += ob_pt_type1.x * (all_parameter->trajectry_weight[0] / sum_weight);
                        sampling_pt.y += ob_pt_type1.y * (all_parameter->trajectry_weight[0] / sum_weight);
                    }
                    if (check_type2) {
                        sampling_pt.x += ob_pt_type2.x * (all_parameter->trajectry_weight[1] / sum_weight);
                        sampling_pt.y += ob_pt_type2.y * (all_parameter->trajectry_weight[1] / sum_weight);
                    }
                    if (check_type3) {
                        sampling_pt.x += ob_pt_type3.x * (all_parameter->trajectry_weight[2] / sum_weight);
                        sampling_pt.y += ob_pt_type3.y * (all_parameter->trajectry_weight[2] / sum_weight);
                    }
                    if (check_type4) {
                        sampling_pt.x += ob_pt_type4.x * (all_parameter->trajectry_weight[3] / sum_weight);
                        sampling_pt.y += ob_pt_type4.y * (all_parameter->trajectry_weight[3] / sum_weight);
                    }
                    if (check_type5) {
                        sampling_pt.x += ob_pt_type5.x * (all_parameter->trajectry_weight[4] / sum_weight);
                        sampling_pt.y += ob_pt_type5.y * (all_parameter->trajectry_weight[4] / sum_weight);
                    }
                    if (dist_robot_ob < std::sqrt(std::pow(sampling_pt.x - robot_pt.x, 2.) + std::pow(sampling_pt.y - robot_pt.y, 2.))) {
                        geometry_msgs::msg::Pose pos;
                        pos.position = p3;
                        PointT p;
                        p.x = p3.x;
                        p.y = p3.y;
                        p.z = p3.z;
                        pcl_cloud->points.push_back(p);
                        point_next_.poses.push_back(pos);  // 残す　＝　step=0がベストだったというだけ
                    }
                }
                else {
                    geometry_msgs::msg::Pose pos;
                    PointT p;
                    p.x = next_pt.x;
                    p.y = next_pt.y;
                    p.z = next_pt.z;
                    pcl_cloud->points.push_back(p);
                    pos.position = next_pt;
                    point_next_.poses.push_back(pos);
                }
            }

            pcl::toROSMsg(*pcl_cloud, cloud_msg);
            cloud_msg.header = header_;
            pub_point1_->publish(point1_);
            pub_point2_->publish(point2_);
            pub_point3_->publish(point3_);
            pub_next_point_->publish(point_next_);
            pub_object_cloud_->publish(cloud_msg);
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_vel_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_object_cloud_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_point1_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_point2_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_point3_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_next_point_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_removed_scan_;
        rclcpp::Client<nav2_msgs::srv::ClearCostmapExceptRegion>::SharedPtr clear_client_;
        rclcpp::TimerBase::SharedPtr timer_;

        // std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseArray>> sub_legs_;
        // std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> sub_scan_;
        // std::shared_ptr<message_filters::Synchronizer<DrspaamScanSyncPolicy>> sync_;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_legs_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        geometry_msgs::msg::Twist robot_vel_;
        sensor_msgs::msg::LaserScan scan_;
        bool start_flag_, scan_flag_;
        geometry_msgs::msg::PoseArray points_base_, object_points_;
        geometry_msgs::msg::PoseArray point1_;
        geometry_msgs::msg::PoseArray point2_;
        geometry_msgs::msg::PoseArray point3_;
        geometry_msgs::msg::PoseArray point_next_;
        std_msgs::msg::Header header_;
        // nav2_msgs::srv::ClearCostmapExceptRegion emp_srv_;
        int counter_;
        std::unique_ptr<ALL_PARAMETER> all_parameter;
    };

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetectNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}