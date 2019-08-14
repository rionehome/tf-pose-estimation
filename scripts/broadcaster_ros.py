#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os

from threading import Lock
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tfpose_ros.msg import *

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
        
        self.tf_lock = Lock()
        self.cv_bridge = CvBridge()
        self.pose_estimator = self.__read_model__()
        self.cv_image = None
        
        rospy.Subscriber('/tfpose_ros/input', Image, self.callback_image, queue_size=1, buff_size=2 ** 24)
        self.pose_pub = rospy.Publisher('/tfpose_ros/output', Poses, queue_size=1)
    
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
        # type:(list,Image)->Poses
        poses = Poses()
        for human in humans:
            pose = Pose()
            for body in human.body_parts:
                body_part = human.body_parts[body]
                key_msg = Keypoint()
                key_msg.part = str(body_part.part_idx)
                key_msg.image_position.x = body_part.x * msg.width
                key_msg.image_position.y = body_part.y * msg.height
                key_msg.score = body_part.score
                pose.keypoints.append(key_msg)
            poses.poses.append(pose)
        return poses
    
    def callback_image(self, msg):
        # type:(Image)->None
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
        
        self.pose_pub.publish(self.humans_to_msg(humans, msg))


if __name__ == '__main__':
    BroadCaster()
    rospy.loginfo('start+')
    rospy.spin()
    rospy.loginfo('finished')
