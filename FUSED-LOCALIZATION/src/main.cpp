#include "fused_localization/FusedLocalization.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<FusedLocalization>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Fused Localization System...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

