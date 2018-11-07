#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import Float64MultiArray
a=0
b=0

def talker():
	global a,b
	pub=rospy.Publisher('teb_x',Float64MultiArray,queue_size = 10)
	rospy.init_node('talker',anonymous=True)
	r=rospy.Rate(10)
	mat=Float64MultiArray()
	mat.data=[0]*2
	while not rospy.is_shutdown():
		a=a+1
		b=b+2
		mat.data = [a,b]
		pub.publish(mat)
		rospy.loginfo(mat.data)
		r.sleep()
if __name__ == '__main__':
	talker()
