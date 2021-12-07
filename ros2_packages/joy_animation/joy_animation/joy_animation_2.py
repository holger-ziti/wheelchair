#!/usr/bin/env python3

import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from random import random, randrange, getrandbits
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from wheelchair_gazebo.digital_filter import DirectForm1Structure

class JoyAnimation2(Node):

    def __init__(self):
        super().__init__('simulation_for_joystick')

        # initial joystick values
        self.joy_x = 0.0
        self.joy_y = 0.0

        self.system_pos = 0.0
        self.system_vel = 0.0 # needed ?

        sampling_rate = 10.0
        self.sampling_period = 1.0/sampling_rate # [s]
        self.filter = DirectForm1Structure(
            float,
            [1.0, -1.366, 0.4697], # a_coefficients[0] is always == 1.0
            [0.04617])

        self.system_vel = self.filter.filter(self.joy_x)

        self.time_horizon = 10.0 # duration of preview time in seconds
        self.time_step = 0.010
        f = .1 # frequency (Hz)
        self.omega = 2 * np.pi * f # angular frequency (Hz)

        self.fig = plt.figure(figsize=(40, 40))
        self.ax = plt.axes(xlim=(-1.2, 1.2), ylim=(-1.0, 5.0))
        self.line, = self.ax.plot([], [], c='0.8', linewidth=1.0)
        self.preview, = self.ax.plot([], [], 'b', linewidth=1.0)
        self.reference, = self.ax.plot([], [], 'o', markerfacecolor="None", markeredgecolor='red', markersize=40)
        self.js_point, = self.ax.plot([], [], 'go', markersize=35)
        self.sys_pos, = self.ax.plot([], [], 'yo', markersize=20)

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        self.ref_publisher = self.create_publisher(Joy, 'joy_ref', 10)

    def get_preview(self): # todo: add param string ['sin', 'cos', ...]
        time_secs = float(self.get_clock().now().nanoseconds) / 1e9  # float seconds
        t = np.arange(time_secs, time_secs+self.time_horizon+self.time_step, self.time_step)
        sin = np.sin(t * self.omega)
        t = t - time_secs # delete time offset
        preview = np.vstack((t, sin))

        #self.get_logger().info(f'sin.shape: {sin.shape}')
        return preview

    def publish_joy_ref(self, x, y):
        joy_reference = Joy()
        joy_reference.header.stamp = self.get_clock().now().to_msg()
        v1 = float(x)
        v2 = float(y)
        joy_reference.axes.append(v1)
        joy_reference.axes.append(v2)
        self.ref_publisher.publish(joy_reference)


    def an_init(self):
        # horizontal line
        line_x = np.linspace(-1.0, 1.0, num=50, endpoint=True)
        line_y = np.zeros(line_x.shape)
        self.line.set_data(line_x, line_y)

        preview = self.get_preview()
        self.preview.set_xdata(preview[1, :])
        self.preview.set_ydata(preview[0, :])
        self.reference.set_xdata(preview[1, 0])
        self.reference.set_ydata(0.0)

        self.publish_joy_ref(preview[1, 0], 0.0)

        self.js_point.set_data([], [])
        self.sys_pos.set_data([], [])
        return self.line, self.reference, self.preview, self.js_point, self.sys_pos

    def an_animate(self, frame):
        preview = self.get_preview()
        self.preview.set_xdata(preview[1, :])
        self.preview.set_ydata(preview[0, :])
        self.reference.set_xdata(preview[1, 0])
        self.reference.set_ydata(0.0)

        self.system_vel = self.filter.filter(self.joy_x)
        self.system_pos = self.system_pos + self.system_vel * self.sampling_period
        self.sys_pos.set_xdata(-self.system_pos)
        self.sys_pos.set_ydata(0.0)

        # joystick movement (left: positive) are inverse to the graphical plot (left: negative) -> invert
        self.js_point.set_data(-self.joy_x, self.joy_y)
        self.js_point.set_data([], [])

        self.publish_joy_ref( preview[1, 0], 0.0)

        return self.line, self.reference, self.preview, self.js_point, self.sys_pos

    def animate(self):
        self.animation = FuncAnimation(
            self.fig,
            self.an_animate,
            init_func=self.an_init,
            frames=200,
            interval=100,  # in ms
            blit=True)  # Remember: When bliting is used, updates of axis ticks are not working correctly.
        plt.show()


    def joy_callback(self, msg):
        self.joy_x = msg.axes[0]
        self.joy_y = 0  # msg.axes[1]


def main(args=None):
    rclpy.init(args=args)

    node = JoyAnimation2()
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
