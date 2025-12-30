#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "../serial_struct.h"
#include <algorithm>
#include <stdint.h>
#include <stdexcept>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "livelybot_msg/msg/motor_state.hpp"

#define my_2pi (6.28318530717f)
#define my_pi (3.14159265358f)

enum motor_type
{
    null,
    _5046,    // 黑色 圆形
    _4538,    // 银色 圆形
    _5047_36, // 5047 双级 36减速比 黑色 方形
    _5047_9,  // 5047 单级 9减速比 黑色 方形
    _4438_32, // 4438 双极 32减速比 黑色 方形
    _4438_8,  // 4438 单极 8减速比 黑色 方形
    _7136_7,  // 
};

enum pos_vel_convert_type
{
    radian_2pi = 0,  // 弧度制
    angle_360,       // 角度制
    turns,           // 圈数
};

class motor
{
private:
    int type, id, num, CANport_num, CANboard_num, iid;
    rclcpp::Node * node_;
    rclcpp::Logger logger_;
    motor_back_t data;
    rclcpp::Publisher<livelybot_msg::msg::MotorState>::SharedPtr motor_pub_;
    livelybot_msg::msg::MotorState p_msg;
    std::string motor_name;
    motor_type type_ = motor_type::null;
    cdc_tr_message_s *p_cdc_tx_message = NULL;
    int id_max = 0;
    int control_type = 0;
    pos_vel_convert_type pos_vel_type = radian_2pi;  


public:
    motor_pos_val_tqe_rpd_s cmd_int16_5param;

    cdc_acm_rx_message_t cmd;
    motor(int _motor_num, int _CANport_num, int _CANboard_num, cdc_tr_message_s *_p_cdc_tx_message, int _id_max, rclcpp::Node * node)
        : CANport_num(_CANport_num), CANboard_num(_CANboard_num), node_(node),
          logger_(node ? node->get_logger() : rclcpp::get_logger("livelybot_motor")),
          p_cdc_tx_message(_p_cdc_tx_message), id_max(_id_max)
    {
        if (node_ == nullptr)
        {
            throw std::runtime_error("motor requires a valid rclcpp::Node pointer");
        }

        motor_name = declare_and_get_param<std::string>("robot/CANboard/No_" + std::to_string(_CANboard_num) +
                                                           "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) +
                                                           "/motor/motor" + std::to_string(_motor_num) + "/name",
                                                       "motor" + std::to_string(_motor_num));

        motor_pub_ = node_->create_publisher<livelybot_msg::msg::MotorState>(
            "/livelybot_real_real/" + motor_name + "_controller/state",
            rclcpp::QoS(10));

        id = declare_and_get_param<int>(
            "robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" +
                std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/id",
            0);
        type = declare_and_get_param<int>(
            "robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" +
                std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/type",
            0);
        num = declare_and_get_param<int>(
            "robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" +
                std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/num",
            0);
        control_type = declare_and_get_param<int>("robot/control_type", 1);
        set_motor_type(type);
        memset(&cmd, 0, sizeof(cmd));
        memset(&data, 0, sizeof(data));
        cmd.motor_cmd.ID = id;
        cmd.head[0] = 0xFE;
        data.ID = id;
        iid = id - 1;
        data.position = 999.0f;
    }
    ~motor() {}
    template <typename T>
    inline T float2int(float in_data, uint8_t type);
    template <typename TT>
    inline TT float2int16(float in_data, uint8_t type);

    inline int16_t pos_float2int(float in_data, uint8_t type);
    inline int16_t vel_float2int(float in_data, uint8_t type);
    inline int16_t tqe_float2int(float in_data, motor_type motor_type);
    inline int16_t rkp_float2int(float in_data, motor_type motor_type);
    inline int16_t rkd_float2int(float in_data, motor_type motor_type);
    inline float pos_int2float(int16_t in_data, uint8_t type);
    inline float vel_int2float(int16_t in_data, uint8_t type);
    inline float tqe_int2float(int16_t in_data, motor_type type);
    inline float pid_scale(float in_data, motor_type motor_type);
    inline int16_t kp_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t ki_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t kd_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t int16_limit(int32_t data);


    void fresh_cmd_int16(float position, float velocity, float torque, float kp, float ki, float kd, float acc, float voltage, float current);

    void position(float position);
    void velocity(float velocity);
    void torque(float torque);
    void voltage(float voltage);
    void current(float current);
    void pos_vel_MAXtqe(float position, float velocity, float torque_max);
    void pos_vel_tqe_kp_kd(float position, float velocity, float torque, float Kp, float Kd);
    void pos_vel_kp_kd(float position, float velocity, float Kp, float Kd);
    void pos_val_acc(float position, float velocity, float acc);
    void pos_vel_rkp_rkd(float position, float velocity, float rKp, float rKd);
    void pos_vel_kp_ki_kd(float position, float velocity, float torque, float kp, float ki, float kd);
    void pos_vel_tqe_rkp_rkd(float position, float velocity, float torque, float rKp, float rKd);

    void fresh_data(int32_t position, int32_t velocity, int32_t torque);
    void fresh_data(int16_t position, int16_t velocity, int16_t torque);
    int get_motor_id() { return id; }
    int get_motor_type() { return type; }
    motor_type get_motor_enum_type() { return type_; }
    int get_motor_num() { return num; }
    /***
     * @brief setting motor type
     * @param type correspond to  different motor type 0~null 1~5046 2~5047_36减速比 3~5047_9减速比
     */
    void set_motor_type(size_t type)
    {
        type_ = static_cast<motor_type>(type);
        // std::cout << "type_:" << type_ << std::endl;
    }
    void set_motor_type(motor_type type)
    {
        type_ = type;
    }
    int get_motor_belong_canport() { return CANport_num; }
    int get_motor_belong_canboard() { return CANboard_num; }
    cdc_acm_rx_message_t *return_cmd_p() { return &cmd; }
    motor_pos_val_tqe_rpd_s *return_pos_val_tqe_rpd_p() { return &cmd_int16_5param; }
    size_t return_size_motor_pos_val_tqe_rpd_s() { return sizeof(motor_pos_val_tqe_rpd_s); }
    motor_back_t *get_current_motor_state() { return &data; }

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