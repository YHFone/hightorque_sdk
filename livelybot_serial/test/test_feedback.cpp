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

class TestFeedbackNode : public rclcpp::Node
{
public:
    TestFeedbackNode() : Node("test_feedback")
    {
        rb_ = std::make_unique<livelybot_serial::robot>(this);
        pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("feedback_formulate", rclcpp::QoS(10));
        RCLCPP_INFO(this->get_logger(), "\033[1;32mSTART\033[0m");
        RCLCPP_INFO(this->get_logger(), "motor num %zu", rb_->Motors.size());

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / 300), std::bind(&TestFeedbackNode::timer_callback, this));  // ~300Hz
    }

private:
    void timer_callback()
    {
        std_msgs::msg::Float32MultiArray feedback_;

        for (motor *m : rb_->Motors)
        {
            if (mode_ == test_mode::kp)
            {
                m->fresh_cmd_int16(kp_mode_pos_, 0.0, 0.0, kp_mode_kp_, 0.0, 0.0, 0, 0, 0);
            }
            else if (mode_ == test_mode::kd)
            {
                m->fresh_cmd_int16(0.0, kd_mode_vel_, 0.0, 0.0, 0, kd_mode_kd_, 0, 0, 0);
            }
            else if (mode_ == test_mode::ff)
            {
                m->fresh_cmd_int16(0.0, 0.0, ff_, 0.0, 0.0, 0, 0, 0, 0);
            }
            else if (mode_ == test_mode::party)
            {
                m->fresh_cmd_int16(kp_mode_pos_, kd_mode_vel_, ff_, kp_mode_kp_, 0, kd_mode_vel_, 0, 0, 0);
            }
        }
        rb_->motor_send_2();

        int idx = 0;
        for (motor *m : rb_->Motors)
        {
            motor_back_t motor = *m->get_current_motor_state();
            if (mode_ == test_mode::kp)
            {
                feedback_.data.push_back((kp_mode_pos_ - motor.position) * kp_mode_kp_);
            }
            else if (mode_ == test_mode::kd)
            {
                feedback_.data.push_back((kd_mode_vel_ - motor.velocity) * kd_mode_kd_);
            }
            else if (mode_ == test_mode::ff)
            {
                feedback_.data.push_back(1);
            }
            else if (mode_ == test_mode::party)
            {
                feedback_.data.push_back((kp_mode_pos_ - motor.position) * kp_mode_kp_ + (kd_mode_vel_ - motor.velocity) * kd_mode_kd_ + ff_);
            }
        }
        pub_->publish(feedback_);

        cont_++;
        RCLCPP_INFO(this->get_logger(), "%d", cont_);
        if (cont_ == 300000)
        {
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Test completed, stopping timer.");
        }
    }

    std::unique_ptr<livelybot_serial::robot> rb_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int cont_ = 0;
    test_mode mode_ = kp;
    float kp_mode_kp_ = 1;
    float kp_mode_pos_ = 0;
    float kd_mode_vel_ = 0;
    float kd_mode_kd_ = 0.1;
    float ff_ = 0.5;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestFeedbackNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
