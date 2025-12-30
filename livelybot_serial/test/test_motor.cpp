#include "rclcpp/rclcpp.hpp"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <condition_variable>
#include <iostream>
#include <thread>

#define Robot_motors 12    //机器人电机总数

class TestMotorNode : public rclcpp::Node {
public:
    TestMotorNode() : Node("test_motor") {
        rb_ = std::make_unique<livelybot_serial::robot>(this);
        RCLCPP_INFO(this->get_logger(), "\033[1;32mSTART\033[0m");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TestMotorNode::timer_callback, this));
    }
private:
    void timer_callback() {
        for (size_t i = 0; i < rb_->Motors.size(); ++i) {
            if (i < Robot_motors / 2) {
                rb_->Motors[i]->pos_vel_MAXtqe(0, 0, 0);  // 这里为了方便出厂测试，直接使用了 pos_vel_MAXtqe 函数（不推荐）
            } else {
                rb_->Motors[i]->pos_vel_MAXtqe(0, 0, 0);  // 这里为了方便出厂测试，直接使用了 pos_vel_MAXtqe 函数（不推荐）
            }
        }
        cont_++;
        if (cont_ == 250) {
            cont_ = 0;
            angle_ *= -1;
        }
        rb_->motor_send_2();
        for (motor *m : rb_->Motors) {
            motor_back_t motor = *m->get_current_motor_state();
            RCLCPP_INFO(this->get_logger(), "ID:%d pos: %8f,vel: %8f,tor: %8f", motor.ID, motor.position, motor.velocity, motor.torque);
        }
    }
    std::unique_ptr<livelybot_serial::robot> rb_;
    rclcpp::TimerBase::SharedPtr timer_;
    int cont_ = 0;
    float angle_ = 0.2;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestMotorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




