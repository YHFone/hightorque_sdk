#include "rclcpp/rclcpp.hpp"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <condition_variable>
#include <iostream>
#include <thread>

//已知脚踝驱动上下两个电机的位置，求脚踝pitch&roll
void ankle_forward_kinematics(double theta_upper, double theta_lower, double &theta_p, double &theta_r){
    double d1 = 0.036;
    double L1 = 0.025;
    double h1 = 0.112;
    double h2 = 0.065;

    double a1, a2, d2;

    a2 = h2;
    a1 = h1;
    d2 = L1;

    double D_L = 2 * d1 * d2;
    double E_L = 2 * L1 * d1 * sin(theta_lower) - 2 * d1 * h2;
    double F_L = 2 * d2 * L1 * sin(theta_lower) - 2 * d2 * h2;
    double G_L = 2 * d1 * d1;
    double H_L = 2 * d2 * L1 * cos(theta_lower);
    double I_L = L1 * L1 + h2 * h2 + 2 * d1 * d1 + d2 * d2 - a2 * a2 - 2 * h2 * L1 * sin(theta_lower);
    double D_R = D_L;
    double E_R = 2 * L1 * d1 * sin(theta_upper) + 2 * d1 * h1;
    double F_R = 2 * d2 * L1 * sin(theta_upper) + 2 * d2 * h1;
    double G_R = G_L;
    double H_R = 2 * d2 * L1 * cos(theta_upper);
    double I_R = L1 * L1 + h1 * h1 + 2 * d1 * d1 + d2 * d2 - a1 * a1 + 2 * h1 * L1 * sin(theta_upper);

    theta_p = 0;
    theta_r = 0;

    int max_iterations = 100;
    double tolerance = 1e-5;

    for (int i = 0; i < max_iterations; ++i) {
        double f1 = -D_L * sin(theta_p) * sin(theta_r) - E_L * sin(theta_r) + F_L * sin(theta_p) - G_L * cos(theta_r) - H_L * cos(theta_p) + I_L;
        double f2 = D_R * sin(theta_p) * sin(theta_r) - E_R * sin(theta_r) - F_R * sin(theta_p) - G_R * cos(theta_r) - H_R * cos(theta_p) + I_R;

        if (fabs(f1) < tolerance && fabs(f2) < tolerance) {
            break;
        }

        double J11 = -D_L * cos(theta_p) * sin(theta_r) + F_L * cos(theta_p) + H_L * sin(theta_p);
        double J12 = -D_L * sin(theta_p) * cos(theta_r) - E_L * cos(theta_r) + G_L * sin(theta_r);
        double J21 = D_R * cos(theta_p) * sin(theta_r) - F_R * cos(theta_p) - H_R * sin(theta_p);
        double J22 = D_R * sin(theta_p) * cos(theta_r) - E_R * cos(theta_r) + G_R * sin(theta_r);

        double det = J11 * J22 - J12 * J21;
        if (fabs(det) < 1e-10) {
            theta_p = 0.0;
            theta_r = 0.0;
            break;
        }

        double J11_inv = J22 / det;
        double J12_inv = -J12 / det;
        double J21_inv = -J21 / det;
        double J22_inv = J11 / det;

        double delta_theta_p = -(J11_inv * f1 + J12_inv * f2);
        double delta_theta_r = -(J21_inv * f1 + J22_inv * f2);

        theta_p = theta_p + delta_theta_p;
        theta_r = theta_r + delta_theta_r;
    }
}


//已知脚踝pitch&roll，求脚踝驱动上下两个电机的位置
void ankle_inverse_kinematics(double theta_p, double theta_r, double &theta_upper, double &theta_lower){
    double d1 = 0.036;

    double L1 = 0.025;

    double h1 = 0.112;

    double h2 = 0.065;


    double A_L = 2 * L1 * L1 * sin(theta_p) - 2 * L1 * d1 * sin(theta_r) - 2 * h2 * L1;
    double A_R = -2 * L1 * L1 * sin(theta_p) - 2 * L1 * d1 * sin(theta_r) + 2 * h1 * L1;
    double B_L = 2 * L1 * L1 * cos(theta_p);
    double B_R = 2 * L1 * L1 * cos(theta_p);
    double C_L = 2 * d1 * d1 + h2 * h2 - h2 * h2 + L1 * L1 + L1 * L1 - 2 * d1 * d1 * cos(theta_r) + 2 * d1 * h2 * sin(theta_r) - 2 * L1 * h2 * sin(theta_p) - 2 * d1 * L1 * sin(theta_p) * sin(theta_r);
    double C_R = 2 * d1 * d1 + h1 * h1 - h1 * h1 + L1 * L1 + L1 * L1 - 2 * d1 * d1 * cos(theta_r) - 2 * d1 * h1 * sin(theta_r) - 2 * L1 * h1 * sin(theta_p) + 2 * d1 * L1 * sin(theta_p) * sin(theta_r);

    double Len_L = A_L * A_L - C_L * C_L + B_L * B_L;
    double Len_R = A_R * A_R - C_R * C_R + B_R * B_R;

    if (Len_L > 0 && Len_R > 0) {
        theta_lower = 2 * atan((-A_L - sqrt(Len_L)) / (B_L + C_L));
        theta_upper = 2 * atan((-A_R + sqrt(Len_R)) / (B_R + C_R));
    } else {
        theta_upper = 0.0;
        theta_lower = 0.0;
    }
}

class TestMotorNode : public rclcpp::Node
{
public:
    TestMotorNode() : Node("test_motor")
    {
        rb_ = std::make_unique<livelybot_serial::robot>(this);
        RCLCPP_INFO(this->get_logger(), "\033[1;32mSTART\033[0m");

        timer_ = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&TestMotorNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        for (motor *m : rb_->Motors)
        {   
            RCLCPP_INFO(this->get_logger(), "id %d pos %f vel %f tqe %f\n", m->get_current_motor_state()->ID, m->get_current_motor_state()->position, m->get_current_motor_state()->velocity, m->get_current_motor_state()->torque);
        }
        ankle_inverse_kinematics(0.3, 0, motor0_, motor1_);
        rb_->Motors[0]->fresh_cmd_int16((float)motor0_, 0, 0, 10, 0, 1, 0, 0, 0);
        rb_->Motors[1]->fresh_cmd_int16((float)motor1_, 0, 0, 10, 0, 1, 0, 0, 0);       
        rb_->motor_send_2();
    }

    std::unique_ptr<livelybot_serial::robot> rb_;
    rclcpp::TimerBase::SharedPtr timer_;
    double motor0_ = 0.0;
    double motor1_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestMotorNode>();
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "END"); 
    rclcpp::shutdown();
    return 0;
}
