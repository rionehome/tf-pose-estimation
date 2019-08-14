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
        rospy.Subscriber("/estimation/output", Poses, self.poses_callback, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        self.bridge = CvBridge()
        self.color_image = None
        self.persons = None
    
    def poses_callback(self, msg):
        # type:(Persons)->None
        self.persons = msg.persons
    
    def image_callback(self, msg):
        # type:(Image)->None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.color_image = copy.copy(cv_image)
        except CvBridgeError as e:
            print 'Cv_Bridge_Error:', e
        
        if self.persons is None:
            cv2.imshow("window", self.color_image)
            cv2.waitKey(1)
            return
        
        person_id = 0
        for person in self.persons:
            for body in person.body_part:
                color_table = {0: (255, 0, 0), 1: (0, 255, 0), 2: (0, 0, 255), 3: (255, 0, 255), 4: (255, 255, 0)}
                cv2.circle(self.color_image, (int(body.x), int(body.y)), 10, (color_table[person_id]),
                           thickness=-1)
            person_id += 1
        
        cv2.imshow("window", self.color_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    Visualize()
    rospy.spin()
