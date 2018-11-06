#! /usr/bin/python

import math


'''
def calc_steps(system, error, feedback):
	steer = 1
	brake = 2
	if system == steer:
		max_swangle = 480
		max_avg_delta = 34.4															# in degrees
		steering_constant = math.fabs(max_swangle) / math.fabs(max_avg_delta)
		error = math.fabs(error)
		max_doable = max_swangle - math.fabs(feedback)
		max_steps = max_doable * steering_constant
		steps = error * steering_constant
		if steps > max_steps:
			steps = max_steps
		else:
			steps = steps
	else:
		steps = 0
	return steps
'''


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
	return steps


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


def calc_error(system, actual, desired):
	steer = 1
	brake = 2
	if system == steer:
		error = float(desired - actual)
	else:
		error = 0.0
	return float(error)
