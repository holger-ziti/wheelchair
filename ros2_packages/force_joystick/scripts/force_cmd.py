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
        self.declare_parameter('voltage_offset_1', '0')
        self.declare_parameter('voltage_offset_2', '0')

        param_1 = self.get_parameter('raw_voltage_number_1').get_parameter_value().integer_value
        param_2 = self.get_parameter('raw_voltage_number_2').get_parameter_value().integer_value
        self.voltage_offset_1 = self.get_parameter('voltage_offset_1').get_parameter_value().integer_value
        self.voltage_offset_2 = self.get_parameter('voltage_offset_2').get_parameter_value().integer_value

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


        self.publish_cmd_vel = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # todo: delete (just for debugging)
        self.publisher_f1_ = self.create_publisher(Float32, 'joystick_force_1', 10)
        self.publisher_f2_ = self.create_publisher(Float32, 'joystick_force_2', 10)
        self.publisher_d1_ = self.create_publisher(Float32, 'damping_force_1', 10)
        self.publisher_d2_ = self.create_publisher(Float32, 'damping_force_2', 10)

        # publish cmd_vel with a fixed frequency
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

        self.joystick_force_1 = 0.0
        self.joystick_force_2 = 0.0

        self.voltage_int1 = 0
        self.voltage_int2 = 0

        self.voltage_to_force_factor_1 = 0.1 # "force to velocity"
        self.voltage_to_force_factor_2 = 0.05 # "force to velocity"

        self.voltage_threshold = 25.0

        # todo: change offset handling
        self.number_of_initial_samples = 100
        self.initial_samples_counter = 0

    def listener_callback_1(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.voltage_int1 = msg.data

    def listener_callback_2(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.voltage_int2 = msg.data

    def timer_callback(self):

        # delete voltage offset
        delta_voltage_1 = self.voltage_int1 - self.voltage_offset_1
        delta_voltage_2 = self.voltage_int2 - self.voltage_offset_2

        # compute force from joystick / delete voltage noise by thresholding
        if abs(delta_voltage_1) < self.voltage_threshold:
            self.joystick_force_1 = 0.0
        else:
            self.joystick_force_1 = self.voltage_to_force_factor_1 * delta_voltage_1
        if abs(delta_voltage_2) < self.voltage_threshold:
            self.joystick_force_2 = 0.0
        else:
            self.joystick_force_2 = self.voltage_to_force_factor_2 * delta_voltage_2

        # compute damping force
        b1 = 0.00 # todo: damping ros parameter
        b2 = 5.00
        damping_force_1 = 0.0 # todo: v near zero -> stop
        damping_force_2 = -sign_float(self.cmd.angular.z) * abs(self.cmd.angular.z) * b2 # todo: use quadratic function?

        # add forces
        force_sum_1 = self.joystick_force_1 + damping_force_1
        force_sum_2 = self.joystick_force_2 + damping_force_2

        # compute delta from force
        v_old = self.cmd.linear.x
        omega_old = self.cmd.angular.z
        delta_v = force_sum_1 * self.timer_period # todo: add inertia factor?
        delta_omega = force_sum_2 * self.timer_period

        if self.publish_cmd_vel:
            # threshold for changes in cmd_vel
            if abs(delta_v) > 0.005:
                self.cmd.linear.x = v_old + delta_v
            else:
                self.cmd.linear.x = v_old
            if abs(delta_omega) > 0.005:
                self.cmd.angular.z = omega_old + delta_omega
            else:
                self.cmd.angular.z = omega_old

            self.publisher_.publish(self.cmd)

        # todo: delete (debugging)
        joystick_force1 = Float32()
        joystick_force1.data = float(self.joystick_force_1)
        self.publisher_f1_.publish(joystick_force1)
        joystick_force2 = Float32()
        joystick_force2.data = float(self.joystick_force_2)
        self.publisher_f2_.publish(joystick_force2)

        damping_force1 = Float32()
        damping_force1.data = float(damping_force_1)
        self.publisher_d1_.publish(damping_force1)
        damping_force2 = Float32()
        damping_force2.data = float(damping_force_2)
        self.publisher_d2_.publish(damping_force2)

        # info: running properly?
        if self.initial_samples_counter <= self.number_of_initial_samples:
            self.initial_samples_counter = self.initial_samples_counter+1
            if self.initial_samples_counter == self.number_of_initial_samples:
                self.get_logger().info(f'voltage1 - mean1: {self.voltage_int1 - self.voltage_offset_1}')
                self.get_logger().info(f'voltage2 - mean2: {self.voltage_int2 - self.voltage_offset_2}')
                self.publish_cmd_vel = True
                self.get_logger().info(f'publishing')


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
