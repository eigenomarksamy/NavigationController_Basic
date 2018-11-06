#! /usr/bin/python


import os
import time
import math
import rospy
import roslib
import smbus
import numpy as np
import skfuzzy as fuzz
from datetime import datetime
from skfuzzy import control as ctrl
from std_msgs.msg import Float32MultiArray


def callback(data):
    global stepper
    i = 0
    for i in range(2):
        stepper[i] = data.data[i]


def callback1(data):
	global velocity_required
	velocity_required = data.data[0]


def listener():
    rospy.Subscriber('arl_stepper_control', Float32MultiArray, callback)
    rospy.Subscriber('arl_velocity_control', Float32MultiArray, callback1)


def calculate_swangle(delta, delta_prev):
	max_steeringwheel = math.radians(480.0)
	max_steeringangle_average = math.radians(34.4)														# in average
	ks = max_steeringangle_average / max_steeringwheel
	#wheel_base = 1.5
	#yaw_difference = yaw_now - yaw_last
	#steering_constant =  max_steeringangle_average / max_steeringwheel
	#steering_wheel = yaw_difference / steering_constant
	#steering_wheel = feedback
	'''
	try:
		rolling_radius_now = wheel_base / yaw_difference
	except ZeroDivisionError as straightline_motion:
		steering_wheel = 0.0
		steering_wheel = feedback
	else:
		steering_wheel = yaw_difference * (1 / steering_constant)
		steering_wheel = feedback
	'''
	return float((delta - delta_prev) /  ks)


def calculate_swangle_rate(yawrate, yawrate_prev):
	max_sw_angle = math.radians(480.0)
	max_wheel_angle = math.radians(34.4)
	wheel_base = 1.5
	ackermann_constant = max_wheel_angle / max_sw_angle
	swangle_rate = yawrate / ackermann_constant
	swangle_rate_prev = yawrate_prev / ackermann_constant
	return float(swangle_rate -swangle_rate_prev)


def calc_steps(system, position, feedback):
	steer = 1
	brake = 2
	if system == steer:
		max_swangle = math.radians(480.0)
		max_avg_delta = math.radians(34.4)															# in degrees
		steps_constant = max_swangle * (max_swangle * 1000.0 / math.radians(360.0))
		if position > 0:
			if feedback > 0:
				max_doable = max_swangle - feedback
			else:
				max_doable = max_swangle
		else:
			if feedback > 0:
				max_doable = max_swangle
			else:
				max_doable = max_swangle - feedback
		max_steps = max_doable * steps_constant
		steps = (position - feedback) * steps_constant
		if steps > max_steps:
			steps = max_steps
		else:
			steps = steps
	else:
		steps = 0
	return math.fabs(steps)


def calc_delay(system, steps_req, rate_of_change):
	steer = 1
	brake = 2
	if system == steer:
		min_delay = 1000											# delay in microseconds
		rate_to_delay = 1000										# (no. of steps per degree)
		m = 0														# acceleration multiplier
		time_to_accelerate = (min_delay * (steps_req / 4))
		time_to_decelerate = time_to_accelerate
		delay_previous = rate_of_change * rate_to_delay
		delay_previous = min_delay
		q = m * delay_previous**2
		delay = delay_previous * (1 + q + q**2)
		if delay < min_delay:
			delay = min_delay
		else:
			delay = delay
	else:
		delay = 0
	return delay


def calc_direction(system, error_req):
	steer = 1
	brake = 2
	direction = 0
	CCW = 1
	CW = 0
	RIGHT = CW
	LEFT = CCW
	if system == steer:
#	    if yaw >= 0 and req_yaw >= 0:
#        	if (yaw - req_yaw) > 0:
#            		direction = RIGHT
#	        else:
#    	       		direction = LEFT
#	    elif yaw > 0 and req_yaw <= 0:
#    	    	if req_yaw > (yaw - 180):
#       	    		direction = RIGHT
#        	else:
#           		direction = LEFT
#            elif yaw < 0 and req_yaw < 0:
#        	if yaw < req_yaw:
#        		direction = LEFT
#        	else:
#        		direction = RIGHT
#            else:
#        	if (yaw - req_yaw) < 180:
#        		direction = LEFT
#        	else:
#        		direction = RIGHT
		if (error_req) > 0:
			direction = RIGHT
		else:
			direction = LEFT
    	else:
    	    direction = 0
        return direction


def steer_control(steer_list):
    delta_yaw = steer_list[0]
    wheel_rate = steer_list[1]
    wheel_position = steer_list[2]
    steer = 1
    max_sw_angle = math.radians(480.0)
    max_tires_angle = math.radians(34.4)
    k_steer = max_tires_angle / max_sw_angle
    sliding_threshold = math.radians(10.0)
    null = 0
    turning = 1
    sliding = 2
    err_in_heading = delta_yaw
    if err_in_heading > 0.0:
        if err_in_heading <= sliding_threshold:
            state = sliding
        else:
            state =turning
    elif err_in_heading < 0.0:
        if err_in_heading >= (-1 * sliding_threshold):
            state = sliding
        else:
            state = turning
    else:
        state = null
    sw_position_req = err_in_heading * k_steer
    direction = calc_direction(steer, err_in_heading)
    steps = calc_steps(steer, err_in_heading, wheel_position)
    delay = calc_delay(steer, steps, wheel_rate)
    return state, direction, steps, delay


def write_to(system, system_parameters):
	bus = smbus.SMBus(1)
	drive_address = 0x05
	steer_address = 0x04
	brake_address = 0x03
	drive_index = 0
	steer_index = 1
	brake_index = 2
	steer_i2c_counter = 0
	if system == drive_index:
		address = drive_address
		drive_state_to_send = byte(system_parameters[0])
		pwm_to_send = byte(system_parameters[1])
		bus.write_i2c_block_data(address, 0, [drive_state_to_send, pwm_to_send])
	elif system == steer_index:
		address = steer_address
		for steer_i2c_counter in range(4):
			if system_parameters[steer_i2c_counter] >= 255:
				system_parameters[steer_i2c_counter] = 254
		steer_state_to_send 	= int(byte(system_parameters[0]))
		steer_direction_to_send = int(byte(system_parameters[1]))
		steer_delay_to_send 	= int(byte(math.ceil(system_parameters[3] / 10)))
		steer_steps_to_send 	= int(byte(system_parameters[2]))
		bus.write_i2c_block_data(address, 0, [steer_state_to_send, steer_direction_to_send, steer_delay_to_send, steer_steps_to_send])
	else:
		address = brake_address


def main():
	global stepper, index, velocity_required
	pwm_out = 0.0
	velocity_required = 0.0
	drive_address = 0x05
	auto_mode = 1
	velocity = ctrl.Antecedent(np.arange(-0.1,  10, 0.1), 'velocity')
	pwm 	 = ctrl.Consequent(np.arange(-1  , 256, 1  ), 'pwm'     )

	velocity['stop']		 = fuzz.trimf(velocity.universe, [0  , 0  , 0.1])
	velocity['slow']         = fuzz.trimf(velocity.universe, [0.1, 0.5, 1  ])
	velocity['medium_slow']  = fuzz.trimf(velocity.universe, [0.5, 3  , 5  ])
	velocity['medium']       = fuzz.trimf(velocity.universe, [3  , 5  , 7  ])
	velocity['medium_fast']  = fuzz.trimf(velocity.universe, [5  , 7  , 9  ])
	velocity['fast']         = fuzz.trimf(velocity.universe, [7  , 9  , 9  ])

	pwm['stop']              = fuzz.trimf(pwm.universe, [  0,  10,  10])
	pwm['slow']              = fuzz.trimf(pwm.universe, [ 10,  35,  80])
	pwm['medium']            = fuzz.trimf(pwm.universe, [ 35, 115, 150])
	pwm['high']              = fuzz.trimf(pwm.universe, [115, 185, 255])

	velocity.view()
	pwm.view()
	stepper 		     = [0.0, 0.0, 0.0, 0.0]
	steer_dependencies   = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
	to_control_steer     = [0.0] * 3
	feedback             = [0.0, 0.0, 0.0, 0.0]
	yaw_err_index        = 0
	yawrate_err_index    = 1
	current_index 	     = 0
	previous_index 	     = 1
	req_index 		     = 2
	previous_req_index   = 3
	current_angle_index  = 0
	current_rate_index 	 = 1
	previous_angle_index = 2
	previous_rate_index  = 3
	steer_parameters     = [0.0] * 4
	steer_system         = 1
	ts                   = 1.0
	while True:
		listener()
		steer_dependencies[yaw_err_index][current_index]     = stepper[0]
		steer_dependencies[yawrate_err_index][current_index] = stepper[1]
		feedback[current_angle_index]			     	     = calculate_swangle(steer_dependencies[yaw_err_index][current_index], steer_dependencies[yaw_err_index][previous_index])
		feedback[current_rate_index]					     = calculate_swangle_rate(steer_dependencies[yawrate_err_index][current_index], steer_dependencies[yawrate_err_index][previous_index])
		to_control_steer[0]                                  = steer_dependencies[yaw_err_index][current_index]
		to_control_steer[1]                                  = feedback[current_rate_index]
		to_control_steer[2]									 = feedback[current_angle_index]
		velocity_input 										 = velocity_required
		steer_parameters[state_index], steer_parameters[direction_index], steer_parameters[steps_index], steer_parameters[delay_index] = steer_control(to_control_steer)
		rule1 = ctrl.Rule(velocity['stop']                                                      , pwm['stop']  )
		rule2 = ctrl.Rule(velocity['slow']                                                      , pwm['slow']  )
		rule3 = ctrl.Rule(velocity['medium_slow'] | velocity['medium'] | velocity['medium_fast'], pwm['medium'])
		rule4 = ctrl.Rule(velocity['fast']                                                      , pwm['high']  )
		velocity_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4])
		controlling = ctrl.ControlSystemSimulation(velocity_ctrl)
		controlling.input['velocity_input']
		controlling.compute()
		print controlling.output['pwm']
	  	pwm_out = controlling.output['pwm']
	  	if pwm_out >= 255:
	  		pwm_out = 254
  		pwm_out   = byte(int(pwm_out))
  		auto_mode = byte(int(auto_mode))
  		bus.write_i2c_block_data(drive_address, 0, [auto_mode, pwm_out])
        	write_to(steer_system, steer_parameters)
        	steer_dependencies[yaw_err_index][previous_index]          = steer_dependencies[yaw_index][current_index]
        	steer_dependencies[yawrate_err_index][previous_index]      = steer_dependencies[yawrate_index][current_index]
        	steer_dependencies[yaw_err_index][previous_req_index]      = steer_dependencies[yaw_index][req_index]
        	steer_dependencies[yawrate_err_index][previous_req_index]  = steer_dependencies[yawrate_index][req_index]
        	feedback[previous_angle_index] 	               			   = feedback[current_angle_index]
        	feedback[previous_rate_index]      	           			   = feedback[current_rate_index]


if __name__ =='__main__':
    main()
