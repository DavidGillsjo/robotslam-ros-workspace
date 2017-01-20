#!/usr/bin/env python

import rospy
import time
import math
import pprint
from sniffer import Sniffer
from std_msgs.msg import (Header, ColorRGBA)
from geometry_msgs.msg import (Point, PointStamped, Pose, Quaternion, Vector3)
from visualization_msgs.msg import Marker
from scapy.all import *
#from std_msgs.msg import String

class WifiScanner:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.t = 0
        self.pub_data = rospy.Publisher('wifi_scanner/data', PointStamped, queue_size=10)
        self.pub_visualization = rospy.Publisher('wifi_scanner/visualization', Marker, queue_size=10)
        self.sniffer = Sniffer("mon0")
        rospy.init_node('wifi_scanner')

    def string_to_color(self, str):
        hash = str.__hash__()
        r = ((hash & 0xFF0000) >> 16) / 255.0
        g = ((hash & 0x00FF00) >> 8) / 255.0
        b = (hash & 0x0000FF) / 255.0
        return r, g, b

    def bssid_to_int32(self, bssid):
        return bssid.__hash__() & (2**31 - 1)

    def rssid_to_distance(self, rssi):
        # RSSI = TxPower - 10 * n * lg(d)
        # n = 2 (in free space)
        # d = 10 ^ ((TxPower - RSSI) / (10 * n))
        txPower = -40 # RSSI at 1m distance
        n = 2.1 # Some constant
        return math.pow(10, (txPower - rssi) / (10.0 * n));

    def scan_callback(self, ssid, bssid, rssi):
        scale_factor = self.rssid_to_distance(rssi)
        print("SSID: %s; Distance: %fm; RSSI = %s" % (ssid, scale_factor, rssi))
        header = Header(seq = self.t, frame_id = "map")
        point = Point(x = self.x, y = self.y, z = self.z)
        orientation = Quaternion(x = 0, y = 0, z = 0, w = 0)
        pose = Pose(position = point, orientation = orientation)
        scale = Vector3(1 * scale_factor, 1 * scale_factor, 1 * scale_factor)
        r, g, b = self.string_to_color(bssid)
        color = ColorRGBA(r = r, g = g, b = b, a = 0.25)
        marker = Marker(header = header, type = Marker.SPHERE, id = self.bssid_to_int32(bssid), pose = pose, action = 0, scale = scale, color = color)
        self.pub_visualization.publish(marker)
        self.t += 1

    def scan(self):
        self.sniffer.start(self.scan_callback)
        #rate = rospy.Rate(2) # 10hz
        #sphere_id = 1
        #t = 0
        while not rospy.is_shutdown():
            #x = 2 * math.cos(t)
            #y = 2 * math.sin(t)
            #z = 0
            #hello_str = "scan: %f %f %f" % (x, y, z)
            #rospy.loginfo(hello_str)
            # header = Header(seq = t, frame_id = "map")

            # Publish actual point
            #point = Point(x = x, y = y, z = z)
            #pointStamped = PointStamped(header = header, point = point)
            #pub_data.publish(pointStamped)

            # Publish visulization point
            # orientation = Quaternion(x = 0, y = 0, z = 0, w = 0)
            # pose = Pose(position = point, orientation = orientation)
            # scale = Vector3(1, 1, 1)
            # color = ColorRGBA(r = 0, g = 1, b = 0, a = 0.5)
            # marker = Marker(header = header, type = Marker.SPHERE, id = sphere_id, pose = pose, action = 0, scale = scale, color = color)
            # pub_visualization.publish(marker)

            #t += math.pi / 12
            rate.sleep()

if __name__ == '__main__':
    try:
        wifi_scanner = WifiScanner()
        wifi_scanner.__init__()
        wifi_scanner.scan()
    except rospy.ROSInterruptException:
        pass
