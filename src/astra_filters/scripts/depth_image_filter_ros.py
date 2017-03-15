#!/usr/bin/env python
"""
Purpose of the file: subscribe to a topic called /image_raw of type sensor_msgs/Image
Apply filter to the resulting image
"""
from __future__ import print_function
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class SubThenFilter:
    def __init__(self):
        self.sub = rospy.Subscriber("/image_raw", Image, self.image_callback, queue_size=1)
        self.pub = rospy.Publisher("/image_filtered", Image, queue_size=1)
        self.bridge = CvBridge()
        self.median_blur_size = 5
        self.use_median_blur = True
        self.use_edge_correction = True

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)

        cv_image = np.nan_to_num(cv_image)
        height, width = cv_image.shape[:2]

        if self.use_edge_correction:
            cv2.rectangle(cv_image, (0, 0), (7, height), (0, 0, 0), cv2.FILLED)
            cv2.rectangle(cv_image, (0, height), (width, height - 15), (0, 0, 0), cv2.FILLED)

        if self.use_median_blur:
            cv_image = cv2.medianBlur(cv_image, self.median_blur_size)

        try:
            msg = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
            data.data = msg.data
            self.pub.publish(data)
            # self.out -= 1
            # if (self.out >= 0):
            #     rospy.loginfo("modified data")
            #     rospy.loginfo(data)

        except CvBridgeError as e:
            rospy.loginfo(e)
            #print(e)

if __name__ == "__main__":
    rospy.init_node("depth_filter", anonymous=True)
    sf = SubThenFilter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
cv2.destroyAllWindows()
