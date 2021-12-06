#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float32, Header
from sensor_msgs.msg import Joy



class Force2JoyPublisher(Node):

    def __init__(self):
        super().__init__('force_to_joy_publisher')

        # default parameter values
        self.declare_parameter('raw_voltage_number_1', '1')
        self.declare_parameter('raw_voltage_number_2', '2')
        self.declare_parameter('voltage_offset_1', '0')
        self.declare_parameter('voltage_offset_2', '0')
        self.declare_parameter('voltage_to_force_factor_1', '0.0')
        self.declare_parameter('voltage_to_force_factor_2', '0.0')

        # https://github.com/ros2/ros2cli/blob/master/ros2param/ros2param/api/__init__.py
        raw_voltage_number_1 = self.get_parameter('raw_voltage_number_1').get_parameter_value().integer_value
        raw_voltage_number_2 = self.get_parameter('raw_voltage_number_2').get_parameter_value().integer_value
        self.voltage_offset_1 = self.get_parameter('voltage_offset_1').get_parameter_value().integer_value
        self.voltage_offset_2 = self.get_parameter('voltage_offset_2').get_parameter_value().integer_value
        self.voltage_to_force_factor_1 = self.get_parameter('voltage_to_force_factor_1').get_parameter_value().double_value
        self.voltage_to_force_factor_2 = self.get_parameter('voltage_to_force_factor_2').get_parameter_value().double_value

        self.subscription = self.create_subscription(
            Int32,
            f'raw_voltage_{raw_voltage_number_1}',
            self.listener_callback_1,
            10)

        self.subscription = self.create_subscription(
            Int32,
            f'raw_voltage_{raw_voltage_number_2}',
            self.listener_callback_2,
            10)

        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        
        self.publisher_f1_ = self.create_publisher(Float32, 'joystick_force_1', 10)
        self.publisher_f2_ = self.create_publisher(Float32, 'joystick_force_2', 10)


        # publish with a fixed frequency
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # initial values
        self.joy = Joy()
        self.joy.header = Header()
        self.joy.axes = []
        self.joy.buttons = []
        self.joystick_force_1 = 0.0
        self.joystick_force_2 = 0.0
        self.voltage_int1 = 0
        self.voltage_int2 = 0

        self.voltage_threshold = 25.0


        #self.number_of_initial_samples = 100
        #self.initial_samples_counter = 0
        self.publish_joy = False
        self.number_of_values_from_cb1 = 0 # make sure there were some messages on the topic
        self.number_of_values_from_cb2 = 0
        self.min_number_of_values = 10

    def listener_callback_1(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.voltage_int1 = msg.data
        self.number_of_values_from_cb1 += 1 # count messages to make sure data is available

    def listener_callback_2(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.voltage_int2 = msg.data
        self.number_of_values_from_cb2 += 1 # count messages to make sure data is available

    def timer_callback(self):

        # delete voltage offset
        delta_voltage_1 = self.voltage_int1 - self.voltage_offset_1
        delta_voltage_2 = self.voltage_int2 - self.voltage_offset_2

        # compute force from joystick (delete voltage noise by thresholding)
        if abs(delta_voltage_1) < self.voltage_threshold:
            self.joystick_force_1 = 0.0
        else:
            self.joystick_force_1 = self.voltage_to_force_factor_1 * delta_voltage_1

        if abs(delta_voltage_2) < self.voltage_threshold:
            self.joystick_force_2 = 0.0
        else:
            self.joystick_force_2 = self.voltage_to_force_factor_2 * delta_voltage_2


        self.joy = Joy()
        #self.joy.header = Header()
        self.joy.header.stamp = self.get_clock().now().to_msg()
        v1 = Float32()
        v1 = float(self.joystick_force_1)
        v2 = Float32()
        v2 = float(self.joystick_force_2)
        self.joy.axes.append(v2)
        self.joy.axes.append(v1)

        if self.publish_joy:
            self.publisher_.publish(self.joy)


        # todo: delete (just for debugging)
        joystick_force1 = Float32()
        joystick_force1.data = float(self.joystick_force_1)
        self.publisher_f1_.publish(joystick_force1)
        joystick_force2 = Float32()
        joystick_force2.data = float(self.joystick_force_2)
        self.publisher_f2_.publish(joystick_force2)

        # start publishing non-zero values, if running properly?
        #if self.initial_samples_counter <= self.number_of_initial_samples:
        #    self.initial_samples_counter = self.initial_samples_counter+1
        #if self.initial_samples_counter >= self.number_of_initial_samples:
        if (self.number_of_values_from_cb1 >= self.min_number_of_values) \
            and (self.number_of_values_from_cb2 >= self.min_number_of_values) \
            and self.publish_joy == False:
                self.get_logger().info(f'voltage1 - mean1: {self.voltage_int1 - self.voltage_offset_1}')
                self.get_logger().info(f'voltage2 - mean2: {self.voltage_int2 - self.voltage_offset_2}')
                self.publish_joy = True
                self.get_logger().info(f'publishing')


def main(args=None):
    rclpy.init(args=args)
    
    force_to_joy_publisher = Force2JoyPublisher()
    rclpy.spin(force_to_joy_publisher)
    force_to_joy_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
