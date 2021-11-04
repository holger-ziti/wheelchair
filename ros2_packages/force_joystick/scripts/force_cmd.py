#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist

def sign_float(a):
    return (a > 0) - (a < 0)

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



        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # todo: delete (just for debugging)
        self.publisher_f1_ = self.create_publisher(Float32, 'force1', 10)
        self.publisher_d1_ = self.create_publisher(Float32, 'damping_force1', 10)

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

        self.force_threshold = 15.0

        # todo: change offset handling
        self.number_of_initial_samples = 100
        self.initial_samples_counter = 0
        self.zero_value_list_1 = []
        self.zero_value_list_2 = []
        self.mean_1 = 743.0 # todo: use ros param
        self.mean_2 = 743.0 # todo: use ros param

    def listener_callback_1(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.voltage_int1 = msg.data

    def listener_callback_2(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.voltage_int2 = msg.data

    def timer_callback(self):
        # todo: use this for testing on wheelchair
        # self.cmd.linear.x = self.force_1 * self.timer_period * self.factor_1
        # self.cmd.angular.z = self.force_2 * self.timer_period * self.factor_2

        a = 0.75 # low-pass filter for force/voltage value
        b1 = 0.50 # damping parameter
        b2 = 0.50

        self.force_1 = self.voltage_int1 - self.mean_1 # a * self.force_1 + (1-a) * (self.voltage_int1 - self.mean_1)
        self.force_2 = self.voltage_int2 - self.mean_2 # a * self.force_2 + (1-a) * (self.voltage_int2 - self.mean_2)

        if abs(self.force_1) < self.force_threshold:
            self.force_1 = 0.0
        if abs(self.force_2) < self.force_threshold:
            self.force_2 = 0.0

        damping_force_1 = -sign_float(self.cmd.linear.x) * b1 # - self.cmd.linear.x * b1
        damping_force_2 = -sign_float(self.cmd.angular.z) * b2  #- self.cmd.angular.z * b2

        self.cmd.linear.x = self.cmd.linear.x + self.timer_period * self.factor_1 * (self.force_1 + damping_force_1)
        self.cmd.angular.z = self.cmd.angular.z + self.timer_period * self.factor_2 * (self.force_2 + damping_force_2)

        # self.cmd.linear.x = (self.force_1 - self.mean_1) * 0.01
        # self.cmd.angular.z = (self.force_2 - self.mean_2) * 0.01

        # do not publish noise values
        # if abs(self.cmd.linear.x) < 0.1:
        #    self.cmd.linear.x = 0.0
        # if abs(self.cmd.angular.z) < 0.1:
        #    self.cmd.angular.z = 0.0

        self.publisher_.publish(self.cmd)

        # todo: delte (debugging)
        joystick_force1 = Float32()
        joystick_force1.data = self.force_1
        self.publisher_f1_.publish(joystick_force1)

        damping_force1 = Float32()
        damping_force1.data = damping_force_1
        self.publisher_d1_.publish(damping_force1)

        # todo: change offset handling
        if self.initial_samples_counter <= self.number_of_initial_samples:
            self.zero_value_list_1.append(self.voltage_int1)
            self.zero_value_list_2.append(self.voltage_int2)
            #self.mean_1 = sum(self.zero_value_list_1)/len(self.zero_value_list_1)
            #self.mean_2 = sum(self.zero_value_list_2)/len(self.zero_value_list_2)
            self.initial_samples_counter = self.initial_samples_counter+1

        if self.initial_samples_counter == self.number_of_initial_samples:
            #self.get_logger().info(f'mean_1: {self.mean_1}')
            #self.get_logger().info(f'mean_2: {self.mean_2}')

            self.get_logger().info(f'voltage1 - mean1: {self.voltage_int1 - self.mean_1}')
            self.get_logger().info(f'voltage2 - mean2: {self.voltage_int2 - self.mean_2}')


        # self.get_logger().info(f'cmd.linear.x start: {self.cmd.linear.x}')


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
