#! /usr/bin/python

import i2c
import controller
import estimator
import time
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
    main()
    rospy.spin()

def main():
	listener()
	steer_dependencies[yaw_err_index][current_index]     = stepper[steer_index][0]
	steer_dependencies[yawrate_err_index][current_index] = stepper[steer_index][1]
	brake_dependencies[position_index][current_index]    = stepper[brake_index][0]
	brake_dependencies[position_index][req_index]	     = stepper[brake_index][1]
	brake_dependencies[us_index][current_index]		     = stepper[brake_index][2]
	feedback[bp_index][current_angle_index]			     = estimator.calculate_bpposition(brake_parameters[steps_index])
	feedback[bp_index][current_rate_index]			     = estimator.calculate_bpvelocity(feedback[bp_index][current_angle_index], feedback[bp_index][previous_angle_index], ts)
	feedback[sw_index][current_angle_index]			     = estimator.calculate_swangle(steer_dependencies[yaw_err_index][current_index], steer_dependencies[yaw_err_index][previous_index])
	feedback[sw_index][current_rate_index]			     = estimator.calculate_swangle_rate(steer_dependencies[yawrate_err_index][current_index], steer_dependencies[yawrate_err_index][previous_index])
	to_control_steer[0]                                  = steer_dependencies[yaw_err_index][current_index]
	to_control_steer[1]                                  = feedback[sw_index][current_rate_index]
	to_control_steer[2]									 = feedback[sw_index][current_angle_index]
	to_control_brake[0]								     = brake_dependencies[position_index][current_index]
	to_control_brake[1]								     = brake_dependencies[position_index][req_index]
	to_control_brake[2]								     = brake_dependencies[us_index][current_index]
	to_control_brake[3]								     = feedback[bp_index][current_rate_index]
	steer_parameters[state_index], steer_parameters[direction_index], steer_parameters[steps_index], steer_parameters[delay_index] = controller.steer_control(to_control_steer)
    brake_parameters[state_index], brake_parameters[direction_index], brake_parameters[steps_index], brake_parameters[delay_index] = controller.brake_control(to_control_brake)
    i2c.write_to(brake_system, brake_parameters)
    i2c.write_to(steer_system, steer_parameters)
    steer_dependencies[yaw_err_index][previous_index]          = steer_dependencies[yaw_index][current_index]
    steer_dependencies[yawrate_err_index][previous_index]      = steer_dependencies[yawrate_index][current_index]
    steer_dependencies[yaw_err_index][previous_req_index]      = steer_dependencies[yaw_index][req_index]
    steer_dependencies[yawrate_err_index][previous_req_index]  = steer_dependencies[yawrate_index][req_index]
    brake_dependencies[position_index][previous_index]     = brake_dependencies[position_index][current_index]
    brake_dependencies[us_index][previous_index]           = brake_dependencies[us_index][current_index]
    brake_dependencies[position_index][previous_req_index] = brake_dependencies[position_index][req_index]
    feedback[bp_index][previous_angle_index]               = feedback[bp_index][current_angle_index]
    feedback[sw_index][previous_angle_index]               = feedback[sw_index][current_angle_index]
    feedback[bp_index][previous_rate_index]                = feedback[bp_index][current_rate_index]
    feedback[sw_index][previous_rate_index]                = feedback[sw_index][current_rate_index]


if __name__ == '__main__':
	global stepper, steer_index, brake_index
	stepper 		     = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
	steer_index          = 0
	brake_index          = 1
	steer_dependencies   = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
	brake_dependencies   = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
	to_control_steer     = [0.0] * 5
	to_control_brake     = [0.0] * 5
	feedback             = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
	yaw_err_index        = 0
	yawrate_err_index    = 1
	position_index 	     = 0
	us_index 		     = 1
	bp_index 		     = 0
	sw_index 		     = 1
	current_index 	     = 0
	previous_index 	     = 1
	req_index 		     = 2
	previous_req_index   = 3
	current_angle_index  = 0
	current_rate_index 	 = 1
	previous_angle_index = 2
	previous_rate_index  = 3
	steer_parameters     = [0.0] * 4
	brake_parameters     = [0.0] * 4
	steer_system         = 1
	brake_system         = 2
	ts                   = 1.0

	while True:
		listener()