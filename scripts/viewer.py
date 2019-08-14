#!/usr/bin/env python
# coding: UTF-8
import copy

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from tfpose_ros.msg import *


class Visualize:
    def __init__(self):
        rospy.init_node("visualize")
        rospy.Subscriber("/tfpose_ros/input", Image, self.image_callback, queue_size=1)
        rospy.Subscriber("/tfpose_ros/output", Poses, self.poses_callback, queue_size=1)
        self.bridge = CvBridge()
        self.color_image = None
    
    def poses_callback(self, msg):
        # type:(Poses)->None
        
        person_id = 0
        for pose in msg.poses:
            for key in pose.keypoints:
                color_table = {0: (255, 0, 0), 1: (0, 255, 0), 2: (0, 0, 255), 3: (255, 0, 255), 4: (255, 255, 0)}
                cv2.circle(self.color_image, (int(key.image_position.x), int(key.image_position.y)), 10,
                           (color_table[person_id]), thickness=-1)
            person_id += 1
        
        cv2.imshow("window", self.color_image)
        cv2.waitKey()
    
    def image_callback(self, msg):
        # type:(Image)->None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.color_image = copy.copy(cv_image)
        except CvBridgeError as e:
            print 'Cv_Bridge_Error:', e


if __name__ == '__main__':
    Visualize()
    rospy.spin()
