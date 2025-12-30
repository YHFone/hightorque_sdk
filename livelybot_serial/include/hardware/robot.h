#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "canboard.h"
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <initializer_list>
#include <fstream>

#include <libserialport.h>
#include <dirent.h>
#include <algorithm>

namespace livelybot_serial
{
    class robot
    {
    private:
        std::string robot_name, Serial_Type, CAN_type, CANboard_type, Serial_allocate;
        int arm_dof, leg_dof, CANboard_num, Seial_baudrate, SDK_version;
        rclcpp::Node * node_;
        rclcpp::Logger logger_;
        std::vector<canboard> CANboards;
        std::vector<std::string> str;
        std::vector<lively_serial *> ser;
        float SDK_version2 = 3.1; // SDK版本

    public:
        std::vector<motor *> Motors;
        // std::vector<std::shared_ptr<canport>> CANPorts;
        std::vector<canport *> CANPorts;
        std::vector<std::thread> ser_recv_threads, send_threads;

        explicit robot(rclcpp::Node * node)
            : node_(node), logger_(node ? node->get_logger() : rclcpp::get_logger("livelybot_robot"))
        {
            if (node_ == nullptr)
            {
                throw std::runtime_error("robot requires a valid rclcpp::Node pointer");
            }

            SDK_version = declare_and_get_param<int>("robot/SDK_version", -1);
            Seial_baudrate = declare_and_get_param<int>("robot/Seial_baudrate", 115200);
            robot_name = declare_and_get_param<std::string>("robot/robot_name", std::string("livelybot"));
            CANboard_num = declare_and_get_param<int>("robot/CANboard_num", 1);
            CANboard_type = declare_and_get_param<std::string>("robot/CANboard_type", std::string("default"));
            Serial_Type = declare_and_get_param<std::string>("robot/Serial_Type", std::string("/dev/ttyUSB"));
            Serial_allocate = declare_and_get_param<std::string>("robot/Serial_allocate", std::string("1for2"));

            if (SDK_version < 0)
            {
                RCLCPP_ERROR(logger_, "Failed to get params SDK_version");
            }
            RCLCPP_INFO(logger_, "\033[1;32mGot params SDK_version: %.1fv\033[0m", SDK_version2);
            RCLCPP_INFO(logger_, "\033[1;32mThe robot name is %s\033[0m", robot_name.c_str());
            RCLCPP_INFO(logger_, "\033[1;32mThe robot has %d CANboards\033[0m", CANboard_num);
            RCLCPP_INFO(logger_, "\033[1;32mThe CANboard type is %s\033[0m", CANboard_type.c_str());
            RCLCPP_INFO(logger_, "\033[1;32mThe Serial type is %s\033[0m", Serial_Type.c_str());
            RCLCPP_INFO(logger_, "\033[1;32mThe Serial allocate type is %s\033[0m", Serial_allocate.c_str());

            init_ser();

            for (size_t i = 1; i <= CANboard_num; i++)
            {
                CANboards.emplace_back(i, &ser, node_);
            }

            for (canboard &cb : CANboards)
            {
                cb.push_CANport(&CANPorts);
            }
            for (canport *cp : CANPorts)
            {
                cp->puch_motor(&Motors);
            }
            set_port_motor_num();
            chevk_motor_connection();

            RCLCPP_INFO(logger_, "\033[1;32mThe robot has %ld motors\033[0m", Motors.size());
            RCLCPP_INFO(logger_, "robot init");
        }
        ~robot()
        {
            set_stop();
            motor_send_2();
            for (auto &thread : ser_recv_threads)
            {
                if (thread.joinable())
                    thread.join();
            }
        }

        void motor_send()
        {
            for (canboard &cb : CANboards)
            {
                cb.motor_send();
                // ROS_INFO("ok");
            }
        }
        void motor_send_2()
        {
            for (canboard &cb : CANboards)
            {
                cb.motor_send_2();
            }
        }

        int serial_pid_vid(const char *name, int *pid, int *vid)
        {
            int r = 0;
            struct sp_port *port;
            
            sp_get_port_by_name(name, &port);
            sp_open(port, SP_MODE_READ);
            if (sp_get_port_usb_vid_pid(port, vid, pid) != SP_OK) 
            {
                r = 1;
            } 
            std::cout << "Port: " << name << ", PID: 0x" << std::hex << *pid << ", VID: 0x" << *vid << std::dec << std::endl;

            // 关闭端口
            sp_close(port);
            sp_free_port(port);

            return r;
        }

        int serial_pid_vid(const char *name)
        {
            int pid, vid;
            int r = 0;
            struct sp_port *port;
            
            sp_get_port_by_name(name, &port);
            sp_open(port, SP_MODE_READ);
            if (sp_get_port_usb_vid_pid(port, &vid, &pid) != SP_OK) 
            {
                r = -1;
            } 
            else 
            {
                switch (vid)
                {
                case (0xCAF1):
                    r = 1;
                    break;
                case (0xCAF2):
                    r = 2;
                    break;
                default:
                    r = -2;
                    break;
                }
            }
            // std::cout << "Port: " << name << ", PID: 0x" << std::hex << pid << ", VID: 0x" << vid << std::dec << std::endl;

            // 关闭端口
            sp_close(port);
            sp_free_port(port);

            return r;
        }

        std::vector<std::string> list_serial_ports(const std::string& full_prefix) 
        {
            std::string base_path = full_prefix.substr(0, full_prefix.rfind('/') + 1);
            std::string prefix = full_prefix.substr(full_prefix.rfind('/') + 1);
            std::vector<std::string> serial_ports;
            DIR *directory;
            struct dirent *entry;

            directory = opendir(base_path.c_str());
            if (!directory)
            {
                std::cerr << "Could not open the directory " << base_path << std::endl;
                return serial_ports; // Return an empty vector if cannot open directory
            }

            while ((entry = readdir(directory)) != NULL)
            {
                std::string entryName = entry->d_name;
                if (entryName.find(prefix) == 0)
                { // Check if the entry name starts with the given prefix
                    serial_ports.push_back(base_path + entryName);
                }
            }

            closedir(directory);

            // Sort the vector in ascending order
            std::sort(serial_ports.begin(), serial_ports.end());

            return serial_ports;
        }

        void init_ser()
        {
            std::vector<std::string> ports = list_serial_ports(Serial_Type);
            for (const std::string& port : ports) 
            {
                if (serial_pid_vid(port.c_str()) > 0)
                {
                    RCLCPP_INFO(logger_, "Serial Port%ld = %s", str.size(), port.c_str());
                    str.push_back(port);
                }
            }

            if ((str.size() < 4 * CANboard_num))
            {
                RCLCPP_ERROR(logger_, "Cannot find the motor serial port, please check if the USB connection is normal.");
                exit(-1);
            }

            if (CANboard_num > 1)
            {
                std::vector<std::string> str1, str2;
                for (size_t i = 0; i < 8; i++)
                {   
                    int vid = serial_pid_vid(str[i].c_str());
                    if (vid == 1)
                    {
                        str1.push_back(str[i]);
                    }
                    else if (vid == 2)
                    {
                        str2.push_back(str[i]);
                    }
                    else
                    {
                        RCLCPP_ERROR(logger_, "Failed to open serial port.");
                        exit(-1);
                    }
                }

                for (size_t i = 0; i < 4; i++)
                {
                    std::swap(str[i], str1[i]);
                    std::swap(str[i + 4], str2[i]);
                }
            }
            std::cout << std::endl;
            for (int i = 0; i < 4*CANboard_num; i++)
            {
                std::cout << str[i] << " " << serial_pid_vid(str[i].c_str()) << std::endl;
            }
            
            for (size_t i = 0; i < str.size(); i++)
            {
                lively_serial *s = new lively_serial(&str[i], Seial_baudrate, 1);
                ser.push_back(s);
                if (SDK_version == 2)
                {
                    /* code */
                    ser_recv_threads.push_back(std::thread(&lively_serial::recv_1for6_42, s));
                }
                else
                {
                    RCLCPP_ERROR(logger_, "SDK_version != 2");
                }
            }
        }
        void test_ser_motor()
        {
            for (lively_serial *s : ser)
            {
                s->test_ser_motor();
            }
        }

        /**
         * @brief 设置每个通道的电机数量，并查询主控板固件版本
         */
        void set_port_motor_num()
        {
            for (canboard &cb : CANboards)
            {
                cb.set_port_motor_num();
            }
        }
        void send_get_motor_state_cmd()
        {
            for (canboard &cb : CANboards)
            {
                cb.send_get_motor_state_cmd();
            }
        }

        void chevk_motor_connection()
        {
            int t = 0;
            int num = 0;
            std::vector<int> board;
            std::vector<int> port;
            std::vector<int> id;

#define MAX_DELAY 10000 // 单位 ms

            RCLCPP_INFO(logger_, "Detecting motor connection");
            while (t++ < MAX_DELAY)
            {
                send_get_motor_state_cmd();
                rclcpp::sleep_for(std::chrono::milliseconds(1));

                num = 0;
                std::vector<int>().swap(board);
                std::vector<int>().swap(port);
                std::vector<int>().swap(id);
                for (motor *m : Motors)
                {
                    if (m->get_current_motor_state()->position != 999.0f)
                    {
                        ++num;
                    }
                    else
                    {
                        board.push_back(m->get_motor_belong_canboard());
                        port.push_back(m->get_motor_belong_canport());
                        id.push_back(m->get_motor_id());
                    }
                }

                if (num == Motors.size())
                {
                    break;
                }

                if (t % 1000 == 0)
                {
                    RCLCPP_INFO(logger_, ".");
                }
            }

            if (num == Motors.size())
            {
                RCLCPP_INFO(logger_, "\033[1;32mAll motor connections are normal\033[0m");
            }
            else
            {
                for (int i = 0; i < Motors.size() - num; i++)
                {
                    RCLCPP_ERROR(logger_, "CANboard(%d) CANport(%d) id(%d) Motor connection disconnected!!!", board[i], port[i], id[i]);
                }
                // exit(-1);
                rclcpp::sleep_for(std::chrono::seconds(5));
            }
        }

        void set_stop()
        {
            for (canboard &cb : CANboards)
            {
                cb.set_stop();
            }
        }

        void set_reset()
        {
            for (canboard &cb : CANboards)
            {
                cb.set_reset();
            }
        }

        void set_reset_zero()
        {
            for (canboard &cb : CANboards)
            {
                cb.set_reset_zero();
            }
        }

        void set_reset_zero(std::initializer_list<int> motors)
        {
            for (auto const &motor : motors)
            {
                int board_id = Motors[motor]->get_motor_belong_canboard() - 1;
                int port_id = Motors[motor]->get_motor_belong_canport() - 1;
                int motor_id = Motors[motor]->get_motor_id();
                RCLCPP_INFO(logger_, "%d, %d, %d", board_id, port_id, motor_id);

                if (CANPorts[port_id]->set_conf_load(motor_id) != 0)
                {
                    RCLCPP_ERROR(logger_, "Motor %d settings restoration failed.", motor);
                    return;
                }
                
                RCLCPP_INFO(logger_, "Motor %d settings have been successfully restored. Initiating zero position reset.", motor);
                if (CANPorts[port_id]->set_reset_zero(motor_id) == 0)
                {
                    RCLCPP_INFO(logger_, "Motor %d reset to zero position successfully, awaiting settings save.null", motor);
                    if (CANPorts[port_id]->set_conf_write(motor_id) == 0)
                    {
                        RCLCPP_INFO(logger_, "Motor %d settings saved successfully.", motor);
                    }
                    else
                    {
                        RCLCPP_ERROR(logger_, "Motor %d settings saved failed.", motor);
                    }
                }
                else
                {
                    RCLCPP_ERROR(logger_, "Motor %d reset to zero position failed.", motor);
                }
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
}
#endif
