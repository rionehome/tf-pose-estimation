#!/usr/bin/env python
# coding: UTF-8
import rospy
from tfpose_ros.msg import Persons, Person, BodyPartElm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')


def callback(data):
	plt.cla()
	for person in data.persons:
		for body in person.body_part:
			center = (int(body.x * data.image_w + 0.5), int(body.y * data.image_h + 0.5))
			# ax.scatter(k.position.x, k.position.y, k.position.z)
		plt.scatter(k.position.x, -k.position.y)
	plt.pause(.001)

	# plt.pause(.01)


rospy.init_node('pose_viewer')
rospy.Subscriber('/pose_estimator/pose_3d', Persons, callback)

rospy.spin()
