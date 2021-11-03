#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist



class CommandVelocityFromForcesPublisher(Node):

    def __init__(self):
        super().__init__('force_command_velocity_publisher')

        # default parameter values
        self.declare_parameter('raw_voltage_number_1', '1')
        self.declare_parameter('raw_voltage_number_2', '2')

        param_1 = self.get_parameter('raw_voltage_number_1').get_parameter_value().integer_value
        param_2 = self.get_parameter('raw_voltage_number_2').get_parameter_value().integer_value

        self.subscription = self.create_subscription(
            Int32,
            f'raw_voltage_{param_1}',
            self.listener_callback_1,
            10)

        self.subscription = self.create_subscription(
            Int32,
            f'raw_voltage_{param_2}',
            self.listener_callback_2,
            10)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # publish cmd_vel with a fixed frequency
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.cmd = Twist()
        self.force_1 = 0.0
        self.force_2 = 0.0

        self.voltage_int1 = 0
        self.voltage_int2 = 0

        self.factor_1 = 0.1 # "force to velocity"
        self.factor_2 = 0.05 # "force to velocity"

        self.force_threshold = 0.1

        # todo: change offset handling
        self.number_of_initial_samples = 100
        self.initial_samples_counter = 0
        self.zero_value_list_1 = []
        self.zero_value_list_2 = []
        self.mean_1 = 0.0
        self.mean_2 = 0.0

    def listener_callback_1(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.voltage_int1 = msg.data

    def listener_callback_2(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.voltage_int2 = msg.data

    def cmd_vel_callback(self, msg):
        self.cmd.linear.x = msg.linear.x
        self.cmd.angular.z = msg.angular.z

    def timer_callback(self):
        # todo: use this for testing on wheelchair
        # self.cmd.linear.x = self.force_1 * self.timer_period * self.factor_1
        # self.cmd.angular.z = self.force_2 * self.timer_period * self.factor_2

        # todo: integration is drifting, use threshold?

        a = 0.75 # low-pass filter for force value
        self.force_1 = a * self.force_1 + (1-a) * (self.voltage_int1 - self.mean_1)

        b = 0.5


        #if abs(self.force_1) > self.force_threshold:
        self.cmd.linear.x = self.cmd.linear.x + self.force_1 * self.timer_period * self.factor_1

        self.force_2 = a * self.force_2 + (1-a) * (self.voltage_int2 - self.mean_2)
        #if abs(self.force_2) > self.force_threshold:
        self.cmd.angular.z = self.cmd.angular.z + self.force_2 * self.timer_period * self.factor_2

        #self.cmd.linear.x = (self.force_1 - self.mean_1) * 0.01
        #self.cmd.angular.z = (self.force_2 - self.mean_2) * 0.01

        # do not publish noise values
        #if abs(self.cmd.linear.x) < 0.1:
        #    self.cmd.linear.x = 0.0
        #if abs(self.cmd.angular.z) < 0.1:
        #    self.cmd.angular.z = 0.0

        self.publisher_.publish(self.cmd)

        # todo: change offset handling
        if (self.initial_samples_counter <= self.number_of_initial_samples):
            self.zero_value_list_1.append(self.voltage_int1)
            self.zero_value_list_2.append(self.voltage_int2)
            self.mean_1 = sum(self.zero_value_list_1)/len(self.zero_value_list_1)
            self.mean_2 = sum(self.zero_value_list_2)/len(self.zero_value_list_2)
            self.initial_samples_counter = self.initial_samples_counter+1
            self.get_logger().info(f'mean_1: {self.mean_1}')
        if (self.initial_samples_counter == self.number_of_initial_samples):
            self.get_logger().info(f'mean_2: {self.mean_2}')
        #self.get_logger().info(f'cmd.linear.x start: {self.cmd.linear.x}')


def main(args=None):
    rclpy.init(args=args)

    force_command_velocity_publisher = CommandVelocityFromForcesPublisher()

    rclpy.spin(force_command_velocity_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    force_command_velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
