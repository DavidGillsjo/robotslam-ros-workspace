#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import time
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
#from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('scanner', PointStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz
    x = 0
    y = 0
    z = 0
    while not rospy.is_shutdown():
        hello_str = "scan: %f %f %f" % (x, y, z)
        rospy.loginfo(hello_str)
        header = Header(seq = x, frame_id = "map")

        point = Point(x = x, y = y, z = z)
        pointStamped = PointStamped(header = header, point = point)
        pub.publish(pointStamped)
        x += 0.1
        y += 0.1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
