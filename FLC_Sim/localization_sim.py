#! /usr/bin/python
import math
import time
import rospy
import roslib
from math import pi
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
def publish(xt, yt, at, xdt, ydt, adt, it):
	pub = rospy.Publisher('sensor_fusion', Float64MultiArray, queue_size = 10)
	rospy.init_node('talker', anonymous=True)
	r = rospy.Rate(10)
	mat = Float64MultiArray()
	mat.data = [0.0] * 7
	mat.data = [it, xt, yt, at, xdt, ydt, adt]
	pub.publish(mat)
	rospy.loginfo(mat.data)
	r.sleep()
x = []
y = []
theta = []
xd = []
yd = []
thetad = []
t = []
i = 0
for i in range(1034):
	t.append(i)
	x.append(i)
	y.append(i)
	theta.append(pi/4)
	xd.append(2.5)
	yd.append(2.5)
	thetad.append(0.1)
theta[0] = 0
theta[-1] = 0
thetad[0] = 0
thetad[-1] = 0
xd[0] = 0
xd[-1] = 0
yd[0] = 0
yd[-1] = 0
j = 0
while not rospy.is_shutdown():
	for j in t:
		publish(x[j], y[j], theta[j], xd[j], yd[j], thetad[j], j)
