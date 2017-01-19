#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import time
import math
from std_msgs.msg import (Header, ColorRGBA)
from geometry_msgs.msg import (Point, PointStamped, Pose, Quaternion, Vector3)
from visualization_msgs.msg import Marker
#from std_msgs.msg import String

def talker():
    pub_data = rospy.Publisher('scanner/data', PointStamped, queue_size=10)
    pub_visualization = rospy.Publisher('scanner/visualization', Marker, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    sphere_id = 1
    t = 0
    while not rospy.is_shutdown():
        x = 2 * math.cos(t)
        y = 2 * math.sin(t)
        z = 0
        hello_str = "scan: %f %f %f" % (x, y, z)
        rospy.loginfo(hello_str)
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
