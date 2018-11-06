#! /usr/bin/python


import math


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


def calculate_bpposition(previous_steps):
	braking_constant = 1
	braking_pedal = previous_steps * braking_constant
	return float(braking_pedal)


def calculate_swangle_rate(yawrate, yawrate_prev):
	max_sw_angle = math.radians(480.0)
	max_wheel_angle = math.radians(34.4)
	wheel_base = 1.5
	ackermann_constant = max_wheel_angle / max_sw_angle
	swangle_rate = yawrate / ackermann_constant
	swangle_rate_prev = yawrate_prev / ackermann_constant
	return float(swangle_rate -swangle_rate_prev)


def calculate_bpvelocity(pedal, pedal_prev, dt):
    return (pedal - pedal_prev) / dt
