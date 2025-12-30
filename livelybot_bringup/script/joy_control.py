#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Teleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')

        # 声明参数
        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_linear_y', 0)
        self.declare_parameter('axis_angular', 6)
        self.declare_parameter('vel_linear', 0.15)
        self.declare_parameter('vel_angular', 0.2)
        self.declare_parameter('config_vel_linear', 0)
        self.declare_parameter('config_vel_angular', 1)
        self.declare_parameter('button', 5)

        # 获取参数
        self.axis_lin_x = self.get_parameter('axis_linear_x').get_parameter_value().integer_value
        self.axis_lin_y = self.get_parameter('axis_linear_y').get_parameter_value().integer_value
        self.axis_ang = self.get_parameter('axis_angular').get_parameter_value().integer_value
        self.vlinear = self.get_parameter('vel_linear').get_parameter_value().double_value
        self.vangular = self.get_parameter('vel_angular').get_parameter_value().double_value
        self.config_vlinear = self.get_parameter('config_vel_linear').get_parameter_value().integer_value
        self.config_vangular = self.get_parameter('config_vel_angular').get_parameter_value().integer_value
        self.ton = self.get_parameter('button').get_parameter_value().integer_value

        # 发布者和订阅者
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.sub = self.create_subscription(Joy, 'joy', self.callback, 1)

    def callback(self, joy):
        twist = Twist()

        if joy.buttons[self.config_vlinear]:
            self.vlinear = joy.axes[4]
            self.get_logger().info(f"vlinear: {self.vlinear:.3f}")
        if joy.buttons[self.config_vangular]:
            self.vangular = joy.axes[4]
            self.get_logger().info(f"vangular: {self.vangular:.3f}")

        if joy.buttons[self.ton]:
            twist.linear.x = joy.axes[self.axis_lin_x] * self.vlinear
            twist.linear.y = joy.axes[self.axis_lin_y] * self.vlinear
            twist.angular.z = joy.axes[self.axis_ang] * self.vangular
            self.get_logger().info(f"linear x y: {twist.linear.x:.3f} {twist.linear.y:.3f} angular: {twist.angular.z:.3f}")
            self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    teleop = Teleop()
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        pass
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
