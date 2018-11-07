#! /usr/bin/python

import rospy
import roslib
from std_msgs.msg import Float64MultiArray

def callbackx(data):
	global x_required
	x_required = data.data


def callbacky(data):
	global y_required
	y_required = data.data


def listener():
	rospy.init_node('subscribe')
	rospy.Subscriber('teb_x', Float64MultiArray, callbackx)
	rospy.Subscriber('teb_y', Float64MultiArray, callbacky)
#	rospy.spin()


global x_required, y_required
x_required = []
y_required = []
i = 0
while i < 10000:
	listener()
#	print x_required.__len__()
#	print y_required.__len__()
#	print x_required
#	print y_required
	i += 1
print x_required.__len__()
print y_required.__len__
