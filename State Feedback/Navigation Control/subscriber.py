#! /usr/bin/python

import rospy
from std_msgs.msg import Float32MultiArray
from   nav_msgs.msg import Odometry
import initializer


def callback(data):
    global localization
    global current_index, x_index, y_index, yaw_index
    localization[x_index][current_index]   = data.data[0]
    localization[y_index][current_index]   = data.data[1]
    localization[yaw_index][current_index] = data.data[2]


def callback1(data):
    global localization
    global yawrate_index, current_index
    localization[yawrate_index][current_index] = data.data[9]


def callback2(msg):
    global localization
    localization[xvelocity_index][current_index] = msg.twist.twist.linear.x
    localization[yvelocity_index][current_index] = msg.twist.twist.linear.y


def callback4(data):
    global localization, final_position
    global x_index, req_index, y_index, yaw_index
    localization[x_index][req_index]   = data.data[0]
    localization[y_index][req_index]   = data.data[1]
    localization[yaw_index][req_index] = data.data[2]
    final_position[x_index]            = data.data[3]
    final_position[y_index]            = data.data[4]


def listener():
    rospy.Subscriber('localization_Fusion', Float32MultiArray, callback)
    rospy.Subscriber('imu_data', Float32MultiArray, callback1)
    rospy.Subscriber('teb', Float32MultiArray, callback4)
    rospy.Subscriber('odom', Odometry, callback2)
