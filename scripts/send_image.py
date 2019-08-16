#!/usr/bin/env python
# coding: UTF-8
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image


def main():
    rospy.init_node("send_image")
    image_pub = rospy.Publisher("/tfpose_ros/input", Image, queue_size=1)
    bridge = CvBridge()
    img = cv2.imread("/home/migly/catkin_ws/src/tf-pose-estimation/images/naikaku.jpg")
    
    msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    time.sleep(1)
    image_pub.publish(msg)


if __name__ == '__main__':
    main()
