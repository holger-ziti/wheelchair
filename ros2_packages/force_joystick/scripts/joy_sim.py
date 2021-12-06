#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import threading
import rclpy
from rclpy.node import Node
from random import random, randrange, getrandbits
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from matplotlib.animation import FuncAnimation

class SimulationForJoystick(Node):

    def __init__(self):
        super().__init__('simulation_for_joystick')

        # default parameter values
        #self.declare_parameter('raw_voltage_number_1', '1')

        # https://github.com/ros2/ros2cli/blob/master/ros2param/ros2param/api/__init__.py
        #raw_voltage_number_1 = self.get_parameter('raw_voltage_number_1').get_parameter_value().integer_value

        # todo: make params
        self.n_indexes = 5
        self.n_seconds = 5

        # initial values
        self.x = 0.0
        self.y = 0.0
        self.last_second = 0
        self.rand_index = 2
        self.possible_values = np.linspace(-1.0, 1.0, num=self.n_indexes, endpoint=True)
        # self.possible_angles = np.linspace(0, np.pi, num=self.n_indexes, endpoint=True)

        self.fig = plt.figure(figsize=(40,40))
        self.ax = plt.axes(xlim=(-1.2, 1.2), ylim=(-0.5, 0.5))
        self.line, = self.ax.plot([], [], c='0.8', linewidth=1.0)
        self.circles, = self.ax.plot([], [], 'o', markerfacecolor="None", markeredgecolor='red', markersize=40)
        self.js_point, = self.ax.plot([], [], 'go', markersize=35)

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        self.ref_publisher = self.create_publisher(Joy, 'joy_ref', 10)

    def compute_new_random_index(self):
        if self.rand_index == 0:
            self.rand_index = 1
        elif self.rand_index == self.n_indexes-1:
            self.rand_index = self.n_indexes-2
        else:
            if bool(getrandbits(1)):
                self.rand_index = self.rand_index+1
            else:
                self.rand_index = self.rand_index-1
        #self.rand_index = randrange(self.n_indexes)

    def an_init(self):
        line_x = np.linspace(-1.0, 1.0, num=50, endpoint=True)
        line_y = np.zeros(line_x.shape)
        self.line.set_data(line_x, line_y)
        #self.get_logger().info(f'line_x: {line_x}')

        possible_x = self.possible_values[self.rand_index]#np.sin(self.possible_values[self.rand_index])
        possible_y = np.zeros(possible_x.shape) #np.cos(self.possible_values[self.rand_index])
        self.circles.set_data(possible_x, possible_y)
        #self.get_logger().info(f'possible_x: {possible_x}, possible_y: {possible_y}')

        self.js_point.set_data([], [])
        return self.line, self.circles, self.js_point

    def an_animate(self, frame):

        possible_x = self.possible_values[self.rand_index] #np.sin(self.possible_values[self.rand_index])
        possible_y = np.zeros(possible_x.shape) # np.cos(self.possible_values[self.rand_index])

        # joystick movement (left: positive) are inverse to the graphical plot (left: negative) -> invert
        self.circles.set_data(-possible_x, possible_y)
        self.js_point.set_data(-self.x, self.y)
        #self.get_logger().info(f'possible_x: {possible_x}, possible_y: {possible_y}')

        return self.line, self.circles, self.js_point

    def animate(self):
        self.animation = FuncAnimation(
            self.fig,
            self.an_animate,
            init_func=self.an_init,
            frames=200,
            interval=100, # in ms
            blit=True) # Remember: When bliting is used, updates of axis ticks are not working correctly.
        plt.show()

    def joy_callback(self, msg):
        self.x = msg.axes[0]
        self.y = 0 # msg.axes[1]
        #self.get_logger().info(f'head: {msg.header.stamp.sec}')

        if msg.header.stamp.sec - self.last_second >= self.n_seconds:
            self.last_second = msg.header.stamp.sec
            self.compute_new_random_index()
        #reference_value = self.possible_values[self.rand_index]
        #self.get_logger().info(f'reference_value: {reference_value}')
        joy_reference = Joy()
        joy_reference.header.stamp = self.get_clock().now().to_msg()
        v1 = Float32()
        v1 = float(self.possible_values[self.rand_index])
        v2 = Float32()
        v2 = float(0)
        joy_reference.axes.append(v1)
        joy_reference.axes.append(v2)
        self.ref_publisher.publish(joy_reference)

def main(args=None):

    rclpy.init(args=args)

    node = SimulationForJoystick()
    try:
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
        spin_thread.start()
    except:
        node.destroy_node()
        rclpy.shutdown()
        raise

    try:
        node.animate()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
