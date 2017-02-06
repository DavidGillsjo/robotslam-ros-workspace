#!/usr/bin/env python

import rospy
# import time
# import math
# import pprint
# import threading
# from tf import (TransformListener, ExtrapolationException)
from std_msgs.msg import (Header)
from geometry_msgs.msg import (Point, PointStamped, Pose, PoseStamped, Quaternion, Vector3)


# from visualization_msgs.msg import Marker
# from std_msgs.msg import String

class Walker:
    def __init__(self):
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.init_node('robotslam_walker')
        self.seq = 0

        # Sleep, since publishing seems to fail otherwise
        rospy.sleep(2)

    def move(self):
        self.seq += 1


        header = Header(frame_id='base_link')

        point = Point(x=1, y=0)
        orientation = Quaternion(x=0, y=0, z=0, w=1)
        pose = Pose(position=point, orientation=orientation)
        rospy.loginfo(PoseStamped(header=header, pose=pose))
        self.pub.publish(PoseStamped(header=header, pose=pose))

        #rospy.sleep(5)

        #point = Point(x=-1, y=0)
        #orientation = Quaternion(x=0, y=0, z=0, w=1)
        #pose = Pose(position=point, orientation=orientation)
        #rospy.loginfo(PoseStamped(header=header, pose=pose))
        #self.pub.publish(PoseStamped(header=header, pose=pose))

        # Empty
        return 0


if __name__ == '__main__':
    walker = Walker()

    walker.__init__()
    walker.move()
