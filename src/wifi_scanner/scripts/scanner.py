#!/usr/bin/env python

import rospy
import time
import math
import pprint
import threading
import tf
from wifi_scanner.msg import (WifiMeasurement, WifiMeasurementArray)
from sniffer import Sniffer
from tf import (TransformListener, ExtrapolationException, Exception)
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
        rospy.init_node('wifi_scanner')
        self.tf = TransformListener()
        self.pub_data = rospy.Publisher('wifi_scanner/data', WifiMeasurement, queue_size=10)
        self.pub_data_filtered = rospy.Publisher('wifi_scanner/data_filtered', WifiMeasurement, queue_size=10)
        self.pub_data_filtered_array = rospy.Publisher('wifi_scanner/data_filtered_array', WifiMeasurementArray, queue_size=10)
        self.pub_visualization = rospy.Publisher('wifi_scanner/visualization', Marker, queue_size=10)
        self.sniffer = Sniffer("mon0")
        self.threads = []
        self.prev_rssis = {}
        self.stored_measurements= {} # don't hate me plzzz
        self.alpha = 0.25

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
        point = Point(x = self.x, y = self.y, z = self.z)

        # Data publish
        measurement = WifiMeasurement(ssid = ssid, bssid = bssid, rssi = rssi, position = point, stamp = rospy.Time.now())
        self.pub_data.publish(measurement)

        # Filtered data publish
        if bssid not in self.prev_rssis:
            self.prev_rssis[bssid] = rssi
        filtered_rssi = self.prev_rssis[bssid] + self.alpha * (rssi - self.prev_rssis[bssid])
        self.prev_rssis[bssid] = filtered_rssi
        measurement = WifiMeasurement(ssid = ssid, bssid = bssid, rssi = filtered_rssi, position = point, stamp = rospy.Time.now())
        self.pub_data_filtered.publish(measurement)

        # Store last measurement (filtered), only one per bssid
        self.stored_measurements[bssid] = measurement 

        # ROS visualization
        scale_factor = self.rssid_to_distance(rssi)
        #print("SSID: %s; Distance: %fm; RSSI = %s" % (ssid, scale_factor, rssi))
        header = Header(seq = self.t, frame_id = "map")
        orientation = Quaternion(x = 0, y = 0, z = 0, w = 0)
        pose = Pose(position = point, orientation = orientation)
        scale = Vector3(1 * scale_factor, 1 * scale_factor, 1 * scale_factor)
        r, g, b = self.string_to_color(bssid)
        color = ColorRGBA(r = r, g = g, b = b, a = 0.25)
        marker = Marker(header = header, type = Marker.SPHERE, id = self.bssid_to_int32(bssid), pose = pose, action = 0, scale = scale, color = color)
        self.pub_visualization.publish(marker)
        self.t += 1

    def scan(self):
        print 'Starting wifi sniffing'
        t = threading.Thread(target=self.sniffer.start, args=(self.scan_callback,))
        self.threads.append(t)
        t.start()
        print 'Starting position caching'
        rate = rospy.Rate(10) # 10hz
        errors = 0
        while not rospy.is_shutdown():
            if self.tf.frameExists("/base_footprint") and self.tf.frameExists("/map"):
                try:
                    t = self.tf.getLatestCommonTime("/map", "/base_footprint")
                    (pos_x, pos_y, pos_z), quaternion = self.tf.lookupTransform("/map", "/base_footprint", t)
                    self.x = pos_x
                    self.y = pos_y
                    self.z = pos_z
                except tf.Exception as e:
                    #pass
                    if errors == 50:
                        print 'Failed to get position.'
                        self.stop()
                        raise e
                    else:
                        print 'Could not get position, retrying...'
                        errors += 1

            # publish stored measurements
            self.pub_data_filtered_array.publish(self.stored_measurements.values())
            self.stored_measurements.clear()
            rate.sleep()
        self.stop()

    def stop(self):
        self.sniffer.stop()

if __name__ == '__main__':
    wifi_scanner = WifiScanner()
    try:
        wifi_scanner.__init__()
        wifi_scanner.scan()
    except rospy.ROSInterruptException:
        wifi_scanner.stop()
