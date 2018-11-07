#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Float64MultiArray	

def callback(data):
	speed=data.data[0]
	distance=data.data[1]
	time=distance/speed
	print("speed    : "+str(speed))   
	print("distance : "+ str(distance))
	print("time     : "+str(time))
	print("-----------------------------------------------")
def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('teb_x', Float64MultiArray , callback) 
	rospy.spin() 
if __name__ == '__main__':
     listener()
