#! /usr/bin/python

import rospy
from std_msgs.msg import Float32MultiArray


def callback(data):
    global stepper, steer_index, brake_index
    i = 0
    for i in range(2):
        stepper[steer_index][i] = data.data[i]
    for i in range(i + 1, 5):
        stepper[brake_index][i - 2] = data.data[i]


def listener():
    rospy.Subscriber('arl_stepper_control', Float32MultiArray, callback)
