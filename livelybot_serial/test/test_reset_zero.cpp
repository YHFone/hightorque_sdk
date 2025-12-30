#include "rclcpp/rclcpp.hpp"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>

class TestResetZeroNode : public rclcpp::Node {
public:
    TestResetZeroNode() : Node("test_reset_zero") {
        rb_ = std::make_unique<livelybot_serial::robot>(this);
        RCLCPP_INFO(this->get_logger(), "\033[1;32mSTART\033[0m");
        rb_->set_reset_zero();  // 全部电机重置零位
        // rb_->set_reset_zero({0, 1, 3, 4});  // 指定电机重置零位，参数为 Motors 的下标
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TestResetZeroNode::timer_callback, this));
        start_time_ = this->now();
    }
private:
    void timer_callback() {
        if ((this->now() - start_time_).seconds() < 1.5) {
            // 延时，方便查看结果的，可去掉
            return;
        }
        int num = 0;
        for (motor *m : rb_->Motors) {
            printf("Motrs[%d]: pos %f, vel %f, tqe %f\n", num++, m->get_current_motor_state()->position, m->get_current_motor_state()->velocity, m->get_current_motor_state()->torque);
            rb_->send_get_motor_state_cmd();
        }
        rb_->motor_send_2();
    }
    std::unique_ptr<livelybot_serial::robot> rb_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestResetZeroNode>();
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "END");
    rclcpp::shutdown();
    return 0;
}
