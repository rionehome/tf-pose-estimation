#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys

from threading import Lock
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tfpose_ros.msg import Persons, Person, BodyPartElm

from tf_pose.estimator import TfPoseEstimator
from tf_pose.networks import model_wh, get_graph_path


class BroadCaster:
    def __init__(self):
        
        rospy.loginfo('initialization+')
        rospy.init_node('TfPoseEstimatorROS', anonymous=True, log_level=rospy.INFO)
        # parameters
        self.model = rospy.get_param('~model', 'cmu')
        self.resolution = rospy.get_param('~resolution', '432x368')
        self.resize_out_ratio = float(rospy.get_param('~resize_out_ratio', '4.0'))
        __image_topic__ = rospy.get_param('~camera', '')
        
        if not __image_topic__:
            rospy.logerr('Parameter \'camera\' is not provided.')
            sys.exit(-1)
        
        self.tf_lock = Lock()
        self.cv_bridge = CvBridge()
        self.pose_estimator = self.__read_model__()
        self.cv_image = None
        
        rospy.Subscriber(__image_topic__, Image, self.callback_image, queue_size=1, buff_size=2 ** 24)
        self.pose_pub = rospy.Publisher('/estimation/output', Persons, queue_size=1)
    
    def __read_model__(self):
        # model読み込み
        print "model読み込み"
        try:
            w, h = model_wh(self.resolution)
            graph_path = get_graph_path(self.model)
            ros_pack = rospkg.RosPack()
            graph_path = os.path.join(ros_pack.get_path('tfpose_ros'), graph_path)
            pose_estimator = TfPoseEstimator(graph_path, target_size=(w, h))
            return pose_estimator
        except Exception as e:
            rospy.logerr('invalid model: %s, e=%s' % (self.model, e))
            sys.exit(-1)
    
    @staticmethod
    def humans_to_msg(humans, msg):
        # type:(list,Image)->Persons
        persons = Persons()
        persons.header = msg.header
        persons.image_w = msg.width
        persons.image_h = msg.height
        
        for human in humans:
            person = Person()
            
            for k in human.body_parts:
                body_part = human.body_parts[k]
                
                body_part_msg = BodyPartElm()
                body_part_msg.part_id = body_part.part_idx
                body_part_msg.x = body_part.x * msg.width
                body_part_msg.y = body_part.y * msg.height
                body_part_msg.confidence = body_part.score
                person.body_part.append(body_part_msg)
            persons.persons.append(person)
        return persons
    
    def callback_image(self, msg):
        
        try:
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr('[tf-pose-estimation] Converting Image Error. ' + str(e))
        
        acquired = self.tf_lock.acquire(False)
        if not acquired:
            return
        
        try:
            humans = self.pose_estimator.inference(self.cv_image, resize_to_default=True,
                                                   upsample_size=self.resize_out_ratio)
        finally:
            self.tf_lock.release()
        
        result = self.humans_to_msg(humans, msg)
        
        self.pose_pub.publish(result)


if __name__ == '__main__':
    BroadCaster()
    rospy.loginfo('start+')
    rospy.spin()
    rospy.loginfo('finished')
