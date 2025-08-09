#!/usr/bin/env python3
import matplotlib
matplotlib.use('TkAgg')  # Ensure GUI backend works

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re

class Visualiser(Node):
    def __init__(self):
        super().__init__('gas_plot_node')
        self.x_data, self.y_data = [], []
        self.start_time = self.get_clock().now()

        # Matplotlib setup
        self.fig, self.ax = plt.subplots()
        self.ln, = self.ax.plot([], [], 'r-')
        
        # Subscriber
        self.create_subscription(String, 'gas_sensor_data', self.odom_callback, 10)
        self.get_logger().info("Im alive")

    def plot_init(self):
        self.ax.set_xlim(0, 100)
        # self.ax
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Gas Value')
        return self.ln,

    def odom_callback(self, msg):
        value = self.parse_data(msg.data)
        if value is not None:
            now = self.get_clock().now()
            elapsed = (now.nanoseconds - self.start_time.nanoseconds) * 1e-9
            self.x_data.append(elapsed)
            self.y_data.append(value)
            if len(self.x_data) > 100:
                self.x_data.pop(0)
                self.y_data.pop(0)

    def parse_data(self, raw):
        match = re.findall(r'(\d+\.?\d*)', raw)
        return float(match[-1]) if len(match) != 1 else None

    def update_plot(self, frame):
        self.ln.set_data(self.x_data, self.y_data)
        if self.x_data:
            self.ax.set_xlim(max(0, self.x_data[0]), self.x_data[-1])
        if self.y_data:
            self.ax.set_ylim(max(self.y_data)+10)
        return self.ln,

def main():
    rclpy.init()
    node = Visualiser()

    ani = FuncAnimation(node.fig, node.update_plot, init_func=node.plot_init, blit=True, interval=100)

    def ros_spin(*args): # idfk
        rclpy.spin_once(node, timeout_sec=0.01)

    timer = node.fig.canvas.new_timer(interval=10)
    timer.add_callback(ros_spin)
    timer.start()

    plt.show()  

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
