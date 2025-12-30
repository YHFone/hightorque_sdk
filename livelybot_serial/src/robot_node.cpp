#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "robot_node.h"
#include <memory>

int main(int argc, char** argv)
{
    // 初始化ROS2节点
    rclcpp::init(argc, argv);

    auto node = std::make_shared<livelybot_serial::robot_node>();

    // 进入ROS2事件循环
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}