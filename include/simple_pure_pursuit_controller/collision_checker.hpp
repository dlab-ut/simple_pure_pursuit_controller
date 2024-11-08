#ifndef COLLISION_CHECKER_HPP
#define COLLISION_CHECKER_HPP

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class CollisionChecker {
  public:
    CollisionChecker(rclcpp::Node *node, double distance_threshold, double width_threshold);

    bool isObstacleDetected() const;

  private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Node *node_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    double distance_threshold_;
    double width_threshold_;
    bool obstacle_detected_;
};

#endif // COLLISION_CHECKER_HPP
