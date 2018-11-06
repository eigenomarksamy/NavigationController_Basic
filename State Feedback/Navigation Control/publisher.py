#! /usr/bin/python


import rospy
import roslib
from std_msgs.msg import Float32MultiArray


def talker_velocity(velocity_req):
	pub = rospy.Publisher('arl_velocity_control',Float32MultiArray,queue_size = 10)
	rospy.init_node('talker',anonymous=True)
	r = rospy.Rate(10)
	mat = Float32MultiArray()
	mat.data = [0.0]
	mat.data = [velocity_req]
	pub.publish(mat)
	rospy.loginfo(mat.data)
	r.sleep()


def talker_stepper(steer_req, brake_req):
	pub = rospy.Publisher('arl_stepper_control',Float32MultiArray,queue_size = 10)
	rospy.init_node('talker',anonymous=True)
	r = rospy.Rate(10)
	mat = Float32MultiArray()
	mat.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	mat.data = [steer_req[0], steer_req[1], brake_req[0], brake_req[1], brake_req[2]]
	pub.publish(mat)
	rospy.loginfo(mat.data)
	r.sleep()
