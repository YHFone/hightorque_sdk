#ifndef _CANBOARD_H_
#define _CANBOARD_H_
#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "canport.h"
#include "rclcpp/rclcpp.hpp"
class canboard
{
private:
    int CANport_num;
    rclcpp::Node * node_;
    rclcpp::Logger logger_;
    std::vector<canport*> CANport;
    // std::vector<motor> motor;
    // std::vector<std::shared_ptr<canport>> CANport;

public:
    canboard(int _CANboard_ID, std::vector<lively_serial *> *ser, rclcpp::Node * node)
        : node_(node), logger_(node ? node->get_logger() : rclcpp::get_logger("livelybot_canboard"))
    {
        if (node_ == nullptr)
        {
            throw std::runtime_error("canboard requires a valid rclcpp::Node pointer");
        }
        CANport_num = declare_and_get_param<int>(
            "robot/CANboard/No_" + std::to_string(_CANboard_ID) + "_CANboard/CANport_num",
            0);
        for (size_t j = 1; j <= CANport_num; j++) // 一个串口对应一个CANport
        {
            CANport.push_back(new canport(j, _CANboard_ID, (*ser)[(_CANboard_ID - 1) * CANport_num + j - 1], node_));
        }
    }
    ~canboard() {}
    int get_CANport_num()
    {
        return CANport_num;
    }
    void push_CANport(std::vector<canport*> *_CANport)
    {
        for (canport *c : CANport)
        {
            _CANport->push_back(c);
        }
    }
    void motor_send()
    {
        for (canport *c : CANport)
        {
            c->motor_send();
        }
    }
    void motor_send_2()
    {
        for (canport *c : CANport)
        {
            c->motor_send_2();
        }
    }

    void set_stop()
    {
        for (canport *c : CANport)
        {
            c->set_stop();
        }
    }

    void set_reset()
    {
        for (canport *c : CANport)
        {
            c->set_reset();
        }
    }

    void set_port_motor_num()
    {
        for (canport *c : CANport)
        {
            c->set_motor_num();
        }
    }
    void send_get_motor_state_cmd()
    {
        for (canport *c : CANport)
        {
            c->send_get_motor_state_cmd();
        }
    }
    void set_reset_zero()
    {
        for (canport *c : CANport)
        {
            if (c->set_conf_load() != 0)
            {
                return;
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
            if (c->set_reset_zero() == 0)
            {
                c->set_conf_write();
            }
            c->set_reset();
            c->motor_send_2();
            rclcpp::sleep_for(std::chrono::seconds(5));
            c->motor_send_2();
        }
    }

private:
    template <typename T>
    T declare_and_get_param(const std::string &name, const T &default_value)
    {
        const std::string normalized = normalize_parameter_name(name);
        if (!node_->has_parameter(normalized))
        {
            node_->declare_parameter<T>(normalized, default_value);
        }

        T value = default_value;
        if (!node_->get_parameter(normalized, value))
        {
            RCLCPP_ERROR(logger_, "Failed to get parameter '%s'", normalized.c_str());
            return default_value;
        }

        return value;
    }

    std::string normalize_parameter_name(const std::string &name) const
    {
        std::string normalized = name;
        std::replace(normalized.begin(), normalized.end(), '/', '.');
        return normalized;
    }
};
#endif