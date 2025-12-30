#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <functional>
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"



namespace livelybot_serial
{

    enum robotType
    {
        _12dof,
        _18dof,
        _20dof
    };

    class robot_node : public rclcpp::Node
    {
    public:
        robot_node()
            : rclcpp::Node("joint_state_listener"),
              rb(this)
        {

            type_ = this->declare_parameter<int>("dof_type_", 0);
            RCLCPP_INFO(this->get_logger(), "type is %d", type_);
            switch (type_)
            {
            case 12:
                rbt = robotType::_12dof;
                map_ = map_12dof;
                break;
            case 18:
                rbt = robotType::_18dof;
                map_ = map_18dof;
                break;
            case 20:
                rbt = robotType::_20dof;
                map_ = map_20dof;
                break;
            default:
                break;
            }

            jointStateName = "walking_motor_goals";

            n_motors = 12;
            rkp = 12.;
            rkd = 0.2;

            using std::placeholders::_1;
            joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                jointStateName,
                rclcpp::QoS(10),
                std::bind(&robot_node::jointStateCallback, this, _1));

        };
        ~robot_node()
        {

        }

        // 回调函数，每当接收到新的JointState消息时就会被调用
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            // `msg`包含了关节名称（name）、位置（position）、速度（velocity）和力矩（effort）
            for (size_t i = 0; i < msg->position.size(); ++i)
            {
                // 对于每个关节，你可以根据需要处理它的状态
                // ROS_INFO("Joint %s - Position: %f, Velocity: %f, Effort: %f",
                //          msg->name[i].c_str(),
                //          msg->position[i],
                //          msg->velocity[i],
                //          msg->effort[i]);
                RCLCPP_INFO(this->get_logger(), "Received %.6f", msg->position[i]);
                rb.Motors[map_[i]]->fresh_cmd_int16(msg->position[i], 0.0, 0.0, rkp, 0, rkd, 0, 0, 0);
                

                // 这里添加代码来分发或处理每个关节的数据
                // 例如，发送给电机控制器等
            }
            rb.motor_send_2();
            for (size_t i = 0; i < n_motors; i++)
            {
                motor_back_t motor;
                motor = *rb.Motors[i]->get_current_motor_state();

                // ROS_INFO_STREAM("ID: " << map_[i] << " Pos: " << motor.position << " torque: " << motor.torque);
            }
        }

    private:


        int type_;//12,18,20
        robotType rbt;
        std::string jointStateName;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
        double rkp, rkd;
        livelybot_serial::robot rb;
        int n_motors;
        std::vector<size_t> map_18dof{0, 1, 2, 3, 4, 5, 9, 10, 11, 12, 13, 14};  // 123456
        std::vector<size_t> map_20dof{0, 1, 2, 3, 4, 5, 10, 11, 12, 13, 14, 15}; // 123456
        std::vector<size_t> map_12dof{0, 1, 5, 2, 4, 3, 6, 7, 11, 8, 10, 9};     // 124653
        std::vector<size_t> map_;
    };

};
