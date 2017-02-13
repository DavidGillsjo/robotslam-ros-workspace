#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import collections
from wifi_scanner.msg import WifiMeasurement
from visualizer_data_container import DataContainers
from utils import *
import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Visualizer(animation.TimedAnimation):
    def __init__(self):
        rospy.init_node('wifi_scanner', anonymous=True)
        self.fig, self.ax = plt.subplots()
        self.ax.set_ylim(-100, 0)
        self.containers = DataContainers()
        self.ani = animation.FuncAnimation(self.fig, self.run_anim, None, interval=33, blit=False)

    def run_anim(self, data):
        now = rospy.Time.now().to_nsec()
        half_minute_ago = (rospy.Time.now() - rospy.Duration(30)).to_nsec()
        self.ax.set_xlim(half_minute_ago, now)
        lines = []
        for container in self.containers:
            lines.append(container.line)
            x_data, y_data = container.get_data()
            if len(x_data) == len(y_data):
                x_data = [ x.to_nsec() for x in x_data ]
                container.line.set_data(x_data, y_data)
        plt.legend(handles = lines)

    def callback(self, measurement):
        container = self.containers.get(ssid = measurement.ssid, bssid = measurement.bssid)
        if container.line is None:
            container.line = Line2D([], [], color = string_to_color(measurement.ssid.encode('punycode')), label=measurement.ssid.encode('punycode'))
            container.line.set_data([], [])
            self.ax.add_line(container.line)
        self.containers.get(ssid = measurement.ssid, bssid = measurement.bssid).put(measurement.stamp, measurement.rssi)

    def listen(self):
        rospy.Subscriber('wifi_scanner/data_filtered', WifiMeasurement, self.callback)
        print('Listening')

if __name__ == '__main__':
    visualizer = Visualizer()
    visualizer.listen()
    plt.show()
