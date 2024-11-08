#include "simple_pure_pursuit_controller/simple_pure_pursuit_controller.hpp"

SimplePurePursuitController::SimplePurePursuitController()
    : Node("simple_pure_pursuit_controller"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
      plan_received_(false) {
    setupParameters();
    plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, std::bind(&SimplePurePursuitController::planCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&SimplePurePursuitController::checkPlanTimeout, this));

    collision_checker_ =
        std::make_shared<CollisionChecker>(this, obstacle_distance_threshold_, obstacle_width_threshold_);
}

void SimplePurePursuitController::setupParameters() {
    lookahead_distance_ = this->declare_parameter("lookahead_distance", 0.5);
    deceleration_distance_ = this->declare_parameter("deceleration_distance", 1.0);
    stop_distance_ = this->declare_parameter("stop_distance", 0.3);
    max_linear_velocity_ = this->declare_parameter("max_linear_velocity", 0.3);
    max_angular_velocity_ = this->declare_parameter("max_angular_velocity", 1.5);
    obstacle_distance_threshold_ = this->declare_parameter("obstacle_distance_threshold", 0.5);
    obstacle_width_threshold_ = this->declare_parameter("obstacle_width_threshold", 0.3);

    this->get_parameter("lookahead_distance", lookahead_distance_);
    this->get_parameter("deceleration_distance", deceleration_distance_);
    this->get_parameter("stop_distance", stop_distance_);
    this->get_parameter("max_linear_velocity", max_linear_velocity_);
    this->get_parameter("max_angular_velocity", max_angular_velocity_);
    this->get_parameter("obstacle_distance_threshold", obstacle_distance_threshold_);
    this->get_parameter("obstacle_width_threshold", obstacle_width_threshold_);
}

void SimplePurePursuitController::planCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty plan.");
        plan_received_ = false;
        return;
    }

    plan_received_ = true;
    last_plan_time_ = this->now();

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose.");
        return;
    }

    nav_msgs::msg::Path pruned_path = prunePath(*msg, robot_pose);

    auto carrot_point = getLookAheadPoint(pruned_path, robot_pose);
    if (!carrot_point.has_value()) {
        RCLCPP_WARN(this->get_logger(), "Could not find a valid carrot point.");
        return;
    }

    auto cmd_vel = calculateVelocityCommand(robot_pose, carrot_point.value());
    cmd_vel_pub_->publish(cmd_vel);
}

bool SimplePurePursuitController::getRobotPose(geometry_msgs::msg::PoseStamped &pose) {
    try {
        auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation = transform.transform.rotation;
        return true;
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        return false;
    }
}

nav_msgs::msg::Path SimplePurePursuitController::prunePath(const nav_msgs::msg::Path &path,
                                                           const geometry_msgs::msg::PoseStamped &robot_pose) {
    nav_msgs::msg::Path pruned_path;
    pruned_path.header = path.header;

    bool point_found = false;
    for (const auto &pose : path.poses) {
        if (euclideanDistance(robot_pose, pose) > lookahead_distance_ || point_found) {
            pruned_path.poses.push_back(pose);
            point_found = true;
        }
    }

    if (pruned_path.poses.empty() && !path.poses.empty()) {
        pruned_path.poses.push_back(path.poses.back());
    }

    return pruned_path;
}

std::optional<geometry_msgs::msg::PoseStamped>
SimplePurePursuitController::getLookAheadPoint(const nav_msgs::msg::Path &path,
                                               const geometry_msgs::msg::PoseStamped &robot_pose) {
    for (const auto &pose : path.poses) {
        double distance = euclideanDistance(robot_pose, pose);
        if (distance >= lookahead_distance_) {
            return pose;
        }
    }

    return !path.poses.empty() ? std::make_optional(path.poses.back()) : std::nullopt;
}

double SimplePurePursuitController::euclideanDistance(const geometry_msgs::msg::PoseStamped &p1,
                                                      const geometry_msgs::msg::PoseStamped &p2) {
    return std::hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
}

geometry_msgs::msg::Twist
SimplePurePursuitController::calculateVelocityCommand(const geometry_msgs::msg::PoseStamped &robot_pose,
                                                      const geometry_msgs::msg::PoseStamped &carrot_point) {
    geometry_msgs::msg::Twist cmd_vel;

    // 前方に障害物がある場合は停止
    if (collision_checker_->isObstacleDetected()) {
        return cmd_vel;
    }

    geometry_msgs::msg::PoseStamped carrot_in_base_link;
    tf2::doTransform(carrot_point, carrot_in_base_link,
                     tf_buffer_.lookupTransform("base_link", "map", tf2::TimePointZero));

    double angle_to_carrot = std::atan2(carrot_in_base_link.pose.position.y, carrot_in_base_link.pose.position.x);
    double distance_to_goal = euclideanDistance(robot_pose, carrot_point);
    RCLCPP_INFO(this->get_logger(), "Distance to goal: %.2f, angle to goal: %.2f", distance_to_goal, angle_to_carrot);

    double adjusted_linear_velocity = max_linear_velocity_;
    if (distance_to_goal < stop_distance_) {
        adjusted_linear_velocity = 0.0;
        RCLCPP_INFO(this->get_logger(), "Stopping the robot.");
    } else if (distance_to_goal < deceleration_distance_) {
        adjusted_linear_velocity *= distance_to_goal / deceleration_distance_;
    }

    cmd_vel.linear.x = std::min(adjusted_linear_velocity, max_linear_velocity_);
    cmd_vel.angular.z = std::clamp(2 * angle_to_carrot, -max_angular_velocity_, max_angular_velocity_);

    return cmd_vel;
}

void SimplePurePursuitController::checkPlanTimeout() {
    if (plan_received_ && (this->now() - last_plan_time_).seconds() > 1.2) {
        RCLCPP_WARN(this->get_logger(), "Plan not received, stopping the robot.");
        publishZeroVelocity();
        plan_received_ = false;
    }
}

void SimplePurePursuitController::publishZeroVelocity() {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
}
