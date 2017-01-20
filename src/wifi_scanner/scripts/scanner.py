#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import time
import math
import pprint
from std_msgs.msg import (Header, ColorRGBA)
from geometry_msgs.msg import (Point, PointStamped, Pose, Quaternion, Vector3)
from visualization_msgs.msg import Marker
from scapy.all import *
#from std_msgs.msg import String

def sniff_callback(pkt):
    if pkt.haslayer(Dot11) and pkt.type == 0:
        addr, rssi = parsePacket(pkt)
        if addr is not None and rssi is not None and pkt.info is not "":
            print("SSID: %s with BSSID: %s has RSSI = %s" % (pkt.info, addr, rssi))

def parsePacket(pkt):
    radiotap_formats = {"TSFT":"Q", "Flags":"B", "Rate":"B",
            "Channel":"HH", "FHSS":"BB", "dBm_AntSignal":"b", "dBm_AntNoise":"b",
            "Lock_Quality":"H", "TX_Attenuation":"H", "dB_TX_Attenuation":"H",
            "dBm_TX_Power":"b", "Antenna":"B",    "dB_AntSignal":"B",
            "dB_AntNoise":"B", "b14":"H", "b15":"B", "b16":"B", "b17":"B", "b18":"B",
            "b19":"BBB", "b20":"LHBB", "b21":"HBBBBBH", "b22":"B", "b23":"B",
            "b24":"B", "b25":"B", "b26":"B", "b27":"B", "b28":"B", "b29":"B",
            "b30":"B", "Ext":"B"}
    if pkt.addr2 is not None:
        # check available Radiotap fields
        field, val = pkt.getfield_and_val("present")
        names = [field.names[i][0] for i in range(len(field.names)) if (1 << i) & val != 0]
        # check if we measured signal strength
        if "dBm_AntSignal" in names:
            # decode radiotap header
            fmt = "<"
            rssipos = 0
            for name in names:
                # some fields consist of more than one value
                if name == "dBm_AntSignal":
                    # correct for little endian format sign
                    rssipos = len(fmt)-1
                fmt = fmt + radiotap_formats[name]
            # unfortunately not all platforms work equally well and on my arm
            # platform notdecoded was padded with a ton of zeros without
            # indicating more fields in pkt.len and/or padding in pkt.pad
            decoded = struct.unpack(fmt, pkt.notdecoded[:struct.calcsize(fmt)])
            return pkt.addr2, decoded[rssipos]
    return None, None

def talker():
    pub_data = rospy.Publisher('scanner/data', PointStamped, queue_size=10)
    pub_visualization = rospy.Publisher('scanner/visualization', Marker, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    sphere_id = 1
    t = 0

    sniff(iface = "mon0", prn = sniff_callback)

    while not rospy.is_shutdown():
        x = 2 * math.cos(t)
        y = 2 * math.sin(t)
        z = 0
        hello_str = "scan: %f %f %f" % (x, y, z)
        #rospy.loginfo(hello_str)
        header = Header(seq = t, frame_id = "map")

        # Publish actual point
        point = Point(x = x, y = y, z = z)
        pointStamped = PointStamped(header = header, point = point)
        pub_data.publish(pointStamped)

        # Publish visulization point
        orientation = Quaternion(x = 0, y = 0, z = 0, w = 0)
        pose = Pose(position = point, orientation = orientation)
        scale = Vector3(1, 1, 1)
        color = ColorRGBA(r = 0, g = 1, b = 0, a = 0.5)
        marker = Marker(header = header, type = Marker.SPHERE, id = sphere_id, pose = pose, action = 0, scale = scale, color = color)
        pub_visualization.publish(marker)

        t += math.pi / 12
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
