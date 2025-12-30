#include "rclcpp/rclcpp.hpp"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <condition_variable>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "std_msgs/msg/float32_multi_array.hpp"

enum test_mode
{
    kp,
    kd,
    ff,
    party
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_feedback");
    rclcpp::Rate rate(300.0);
    livelybot_serial::robot rb(node.get());
    auto pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("feedback_formulate", rclcpp::QoS(10));
    RCLCPP_INFO(node->get_logger(), "\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    int cont = 0;
    test_mode mode_=kp;
    std::vector<std::string> motor_name{"null","5046","4538","5047_36","5047_9"}; 
    RCLCPP_INFO(node->get_logger(), "motor num %zu" ,rb.Motors.size());
   float kp_mode_kp=1;
   float kp_mode_pos=0;
   float kd_mode_vel=0;
   float kd_mode_kd=0.1;
   float ff_=0.5;
   
    while (rclcpp::ok()) // 此用法为逐个电机发送控制指令
    {
        /////////////////////////send
        std_msgs::msg::Float32MultiArray feedback_;

        for (motor *m : rb.Motors)
        {
            if(mode_==test_mode::kp)
            {
                m->fresh_cmd_int16(kp_mode_pos, 0.0, 0.0, kp_mode_kp, 0.0, 0.0, 0, 0, 0);
            }
            else if (mode_==test_mode::kd)
            {
                m->fresh_cmd_int16(0.0, kd_mode_vel, 0.0, 0.0, 0, kd_mode_kd, 0, 0, 0);
            }
            else if(mode_==test_mode::ff)
            {
                m->fresh_cmd_int16(0.0, 0.0, ff_, 0.0, 0.0, 0, 0, 0, 0);
            }
            else if(mode_==test_mode::party)
            {
                m->fresh_cmd_int16(kp_mode_pos, kd_mode_vel, ff_, kp_mode_kp, 0, kd_mode_vel, 0, 0, 0);
            }
        }
        rb.motor_send_2();
        
        ////////////////////////recv
        int idx=0;
        for (motor *m : rb.Motors)
        {
            motor_back_t motor;
            motor = *m->get_current_motor_state();
            if(mode_==test_mode::kp)
            {
                feedback_.data.push_back((kp_mode_pos-motor.position)*kp_mode_kp);
            }
            else if (mode_==test_mode::kd)
            {
                feedback_.data.push_back((kd_mode_vel-motor.velocity)*kd_mode_kd);
            }
            else if(mode_==test_mode::ff)
            {
                feedback_.data.push_back(1);
            }
            else if(mode_==test_mode::party)
            {
                feedback_.data.push_back((kp_mode_pos-motor.position)*kp_mode_kp+(kd_mode_vel-motor.velocity)*kd_mode_kd+ff_);
            }
            // ROS_INFO_STREAM(" ID: " << motor.ID << 
            //                 " Pos: " << motor.position <<
            //                 " torque: " << motor.torque<<
            //                 " type: " << motor_name[static_cast<int>(rb.Motors[idx++]->get_motor_enum_type())]
            //                 );
        }
        pub->publish(feedback_);
        
        cont++;
        RCLCPP_INFO(node->get_logger(), "%d",cont);
        if (cont==300000)
        {
            break;
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
