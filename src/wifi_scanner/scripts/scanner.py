#!/usr/bin/env python

import rospy
import threading
import tf
from wifi_scanner.msg import (WifiMeasurement, WifiMeasurementArray)
from sniffer import Sniffer
from tf import (TransformListener, Exception)
from geometry_msgs.msg import (Point)


class WifiScanner:
    def __init__(self):
        rospy.init_node('wifi_scanner')

        self.x = 0
        self.y = 0
        self.z = 0
        self.tf = TransformListener()

        self.pub_data = rospy.Publisher('wifi_scanner/data', WifiMeasurement, queue_size=10)
        self.pub_data_filtered = rospy.Publisher('wifi_scanner/data_filtered', WifiMeasurement, queue_size=10)

        self.publisher_array = rospy.Publisher(
            'wifi_scanner/data_filtered_array',
            WifiMeasurementArray,
            queue_size=10)

        self.sniffer = Sniffer("mon0")
        self.threads = []
        self.prev_rssis = {}
        self.stored_measurements = {}  # don't hate me plzzz
        self.alpha = 0.25
        self.errors = 0

    def scan_callback(self, ssid, bssid, rssi):
        point = Point(x=self.x, y=self.y, z=self.z)

        # Data publish
        measurement = WifiMeasurement(ssid=ssid, bssid=bssid, rssi=rssi, position=point, stamp=rospy.Time.now())
        self.pub_data.publish(measurement)

        # Store last measurement (filtered), only one per bssid
        self.stored_measurements[bssid] = measurement

        # Filtered data publish
        if bssid not in self.prev_rssis:
            self.prev_rssis[bssid] = rssi

        filtered_rssi = self.prev_rssis[bssid] + self.alpha * (rssi - self.prev_rssis[bssid])
        self.prev_rssis[bssid] = filtered_rssi
        measurement = WifiMeasurement(ssid=ssid, bssid=bssid, rssi=filtered_rssi, position=point,
                                      stamp=rospy.Time.now())
        self.pub_data_filtered.publish(measurement)

    def scan(self):
        print 'Starting wifi sniffing'
        t = threading.Thread(target=self.sniffer.start, args=(self.scan_callback,))
        self.threads.append(t)
        t.start()

        print 'Starting position caching'
        rate = rospy.Rate(1)  # 1hz
        self.errors = 0

        while not rospy.is_shutdown():
            # Ensure we have valid reference frames
            if self.tf.frameExists("/base_footprint") and self.tf.frameExists("/map"):
                self.lookupTransform()

            # publish stored measurements
            measurements_array = WifiMeasurementArray(
                measurements=self.stored_measurements.values(),
                stamp=rospy.Time.now(),
                position=Point(x=self.x, y=self.y, z=self.z))

            self.publisher_array.publish(measurements_array)

            self.stored_measurements.clear()
            rate.sleep()

        self.stop()

    def lookupTransform(self):
        try:
            t = self.tf.getLatestCommonTime("/map", "/base_footprint")
            (pos_x, pos_y, pos_z), quaternion = self.tf.lookupTransform("/map", "/base_footprint", t)
            self.x = pos_x
            self.y = pos_y
            self.z = pos_z
        except tf.Exception as e:
            # Disable error terminate
            if False:  # self.errors == 50:
                print 'Failed to get position.'
                self.stop()
                raise e
            else:
                print 'Could not get position, retrying...'
                self.errors += 1

    def stop(self):
        self.sniffer.stop()


if __name__ == '__main__':
    wifi_scanner = WifiScanner()
    try:
        wifi_scanner.__init__()
        wifi_scanner.scan()
    except rospy.ROSInterruptException:
        wifi_scanner.stop()
