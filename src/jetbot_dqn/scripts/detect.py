#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
import cvlib as cv
from cvlib.object_detection import draw_bbox
import numpy as np
import sys

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import roslib
import rospy


class Detect(object):
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/robot/camera1/image_raw",Image,self.camera_callback)

    def camera_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        bbox, label, conf = cv.detect_common_objects(cv_image, enable_gpu=True, confidence=0.7, model='yolov3-tiny')
        bbox_image = draw_bbox(cv_image, bbox, label, conf, write_conf=False)
        cv2.imshow("Image window", bbox_image)
        cv2.waitKey(1)


def main():
    rospy.init_node('detect_node', anonymous=True)
    detect = Detect()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()