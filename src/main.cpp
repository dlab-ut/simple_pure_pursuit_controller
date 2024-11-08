#include "simple_pure_pursuit_controller/simple_pure_pursuit_controller.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePurePursuitController>());
    rclcpp::shutdown();
    return 0;
}
