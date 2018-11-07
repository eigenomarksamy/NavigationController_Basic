#! /usr/bin/python
import rospy
import roslib
from std_msgs.msg import Float64MultiArray
global t, x, y, a, xd, yd, ad
t = 0.0
x = []
y = []
a = []
xd = []
yd = []
ad = []
t_prev = -1
def printV():
	global t, x, y, a, xd, yd, ad
	t = int(t)
	print "t", t
	print "x", x[t]
	print "y", y[t]
	print "a", a[t]
	print "xd", xd[t]
	print "yd", yd[t]
	print "ad", ad[t]
def callback(data):
	global t, x, y, a, xd, yd, ad, t_prev
	t = data.data[0]
	x.append(data.data[1])
	y.append(data.data[2])
	a.append(data.data[3])
	xd.append(data.data[4])
	yd.append(data.data[5])
	ad.append(data.data[6])
	printV()
#	while t.__len__() < 10:
#	t.append(data.data[0])
#	x.append(data.data[1])
#	y.append(data.data[2])
#	a.append(data.data[3])
#	xd.append(data.data[4])
#	yd.append(data.data[5])
#	ad.append(data.data[6])
#	t_prev = t
#	print "t", t
#	print "x", x
#	print "y", y
#	print "a", a
#	print "xd", xd
#	print "yd", yd
#	print "ad", ad
#	print "t_prev", t_prev
def listener():
	rospy.init_node('subscribe')
#	r = rospy.Rate(10)
	rospy.Subscriber('sensor_fusion', Float64MultiArray, callback)
#	r.sleep()
#	rospy.spin()
#	rospy.spin()
if __name__ == '__main__':
#	while True:
	listener()
	rospy.spin()
#	print x_required.__len__()
#	print y_required.__len__()
#	print x_required
#	print y_required
#	if t == t_prev:
#		break
#	else:
#		t_prev = t
#	print t
#	print "X"
#	print x[0]
#	print x[1]
#	print x[2]
#	print x[3]
#	print x[4]
#	print x[5]
#	print x[6]
#	print x[7]
#	print x[8]
#	print x[9]
#	print "--------------"
#	print "Y"
#	print y
#	print "--------------"
#	print a
#	print xd
#	print yd
#	print ad
