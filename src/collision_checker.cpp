#include "simple_pure_pursuit_controller/collision_checker.hpp"

CollisionChecker::CollisionChecker(rclcpp::Node *node, double distance_threshold, double width_threshold)
    : node_(node), distance_threshold_(distance_threshold), width_threshold_(width_threshold),
      obstacle_detected_(false) {
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), std::bind(&CollisionChecker::scanCallback, this, std::placeholders::_1));
}

bool CollisionChecker::isObstacleDetected() const { return obstacle_detected_; }

void CollisionChecker::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    obstacle_detected_ = false;

    auto ranges = msg->ranges;
    double angle_min = msg->angle_min;
    // double angle_max = msg->angle_max;
    double angle_increment = msg->angle_increment;
    // RCLCPP_INFO(node_->get_logger(), "ranges size: %d", ranges.size());

    // 前方の中央インデックス
    int center_index = static_cast<int>(ranges.size() / 2);

    // 前方の一定の角度範囲のスキャンポイントを確認
    int scan_angle_deg = 60;
    int scan_angle_idx = scan_angle_deg * M_PI / 180 / angle_increment; // インデックス数
    // RCLCPP_INFO(node_->get_logger(), "center_index: %d, scan_angle_idx: %d", center_index, scan_angle_idx);

    int start_index = center_index - scan_angle_idx;
    int end_index = center_index + scan_angle_idx;
    // RCLCPP_INFO(node_->get_logger(), "start_index: %d, end_index: %d", start_index, end_index);

    // 障害物の検出処理
    for (int i = start_index; i <= end_index; ++i) {
        double range = msg->ranges[i];
        double angle = i * angle_increment + angle_min;
        double x = range * cos(angle);
        double y = range * sin(angle);
        // RCLCPP_INFO(node_->get_logger(), "angle: %f, x: %f, y: %f, i: %d, range: %f", angle, x, y, i, range);

        // 前方の指定距離以内 & 横幅の指定範囲内
        if (x < distance_threshold_ && std::abs(y) < width_threshold_) {
            obstacle_detected_ = true;
            // RCLCPP_INFO(node_->get_logger(), "Obstacle x: %f, y: %f, i: %d, range: %f", x, y, i, range);
            RCLCPP_INFO(node_->get_logger(), "Obstacle detected, stopping robot.");
            break;
        }
    }
}
