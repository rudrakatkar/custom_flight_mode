#include <rclcpp/rclcpp.hpp>
#include "px4_swarm_drawer/modes.hpp"
#include <px4_ros2/components/node_with_mode.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Usage: node <drone_id_1_2_3>");
        return 1;
    }
    
    int drone_id = std::stoi(argv[1]);
    const bool kEnableDebugOutput = true;

    if (drone_id == 1) {
        auto node = std::make_shared<px4_ros2::NodeWithMode<DrawStraightLineMode>>("draw_mode_1", kEnableDebugOutput);
        rclcpp::spin(node);
    } 
    else if (drone_id == 2) {
        auto node = std::make_shared<px4_ros2::NodeWithMode<DrawCurveMode>>("draw_mode_2", kEnableDebugOutput);
        rclcpp::spin(node);
    } 
    else if (drone_id == 3) {
        auto node = std::make_shared<px4_ros2::NodeWithMode<DrawDiagonalMode>>("draw_mode_3", kEnableDebugOutput);
        rclcpp::spin(node);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Invalid drone id: %d", drone_id);
    }

    rclcpp::shutdown();
    return 0;
}
