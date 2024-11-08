#ifndef SIMPLE_PURE_PURSUIT_CONTROLLER_HPP
#define SIMPLE_PURE_PURSUIT_CONTROLLER_HPP

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "collision_checker.hpp"

class SimplePurePursuitController : public rclcpp::Node {
  public:
    SimplePurePursuitController();

  private:
    void setupParameters();
    void planCallback(const nav_msgs::msg::Path::SharedPtr msg);
    bool getRobotPose(geometry_msgs::msg::PoseStamped &pose);
    nav_msgs::msg::Path prunePath(const nav_msgs::msg::Path &path, const geometry_msgs::msg::PoseStamped &robot_pose);
    std::optional<geometry_msgs::msg::PoseStamped> getLookAheadPoint(const nav_msgs::msg::Path &path,
                                                                     const geometry_msgs::msg::PoseStamped &robot_pose);
    double euclideanDistance(const geometry_msgs::msg::PoseStamped &p1, const geometry_msgs::msg::PoseStamped &p2);
    geometry_msgs::msg::Twist calculateVelocityCommand(const geometry_msgs::msg::PoseStamped &robot_pose,
                                                       const geometry_msgs::msg::PoseStamped &carrot_point);
    void checkPlanTimeout();
    void publishZeroVelocity();

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;
    bool plan_received_;
    rclcpp::Time last_plan_time_;

    double lookahead_distance_;
    double deceleration_distance_;
    double stop_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;

    std::shared_ptr<CollisionChecker> collision_checker_;
    double obstacle_distance_threshold_;
    double obstacle_width_threshold_;
};

#endif // SIMPLE_PURE_PURSUIT_CONTROLLER_HPP
