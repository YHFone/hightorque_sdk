#include "rclcpp/rclcpp.hpp"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <condition_variable>
#include <iostream>
#include <thread>

#define Robot_motors 12    //机器人电机总数

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_motor");
    rclcpp::Rate rate(100.0);
    livelybot_serial::robot rb(node.get());
    RCLCPP_INFO(node->get_logger(), "\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    int cont = 0, i = 0;
    float angle = 0.2;
    while (rclcpp::ok()) // 此用法为逐个电机发送控制指令
    {
        /////////////////////////send
        for (motor *m : rb.Motors)
        {
            if(i < (int)Robot_motors/2)
            {
                rb.Motors[i]->pos_vel_MAXtqe(0, 0, 0);  // 这里为了方便出厂测试，直接使用了 pos_vel_MAXtqe 函数（不推荐）
            }
            else
            {
                rb.Motors[i]->pos_vel_MAXtqe(0, 0, 0);  // 这里为了方便出厂测试，直接使用了 pos_vel_MAXtqe 函数（不推荐）
            }
            i++;
        }
        i = 0;
        cont++;
        if(cont==250)
        {
            cont = 0;
            angle*=-1;
        }
        rb.motor_send_2();
        ////////////////////////recv
        for (motor *m : rb.Motors)
        {
            motor_back_t motor;
            motor = *m->get_current_motor_state();
            RCLCPP_INFO(node->get_logger(), "ID:%d pos: %8f,vel: %8f,tor: %8f", motor.ID, motor.position, motor.velocity, motor.torque);
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}




