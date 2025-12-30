#include "rclcpp/rclcpp.hpp"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_reset_zero");
    rclcpp::Rate rate(100.0);
    livelybot_serial::robot rb(node.get());

    RCLCPP_INFO(node->get_logger(), "\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    rb.set_reset_zero();  // 全部电机重置零位
    // rb.set_reset_zero({0, 1, 3, 4});  // 指定电机重置零位，参数为 Motors 的下标

    rclcpp::sleep_for(std::chrono::milliseconds(1500));  // 延时，方便查看结果的，可去掉
    while (rclcpp::ok()) // 此用法为逐个电机发送控制指令
    {
        /////////////////////////send
        int num = 0;
        for (motor *m : rb.Motors)
        {   
            printf("Motrs[%d]: pos %f, vel %f, tqe %f\n", num++, m->get_current_motor_state()->position, m->get_current_motor_state()->velocity, m->get_current_motor_state()->torque);
            rb.send_get_motor_state_cmd();
        }
        rb.motor_send_2();

        rclcpp::spin_some(node);
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "END");
    rclcpp::shutdown();
    return 0;
}
