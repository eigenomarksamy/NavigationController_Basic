#! /usr/bin/python


import math


def calculate_req_yaw(x, y):
	theta = math.atan2(y, x)
	return float(theta)


def calculate_req_velocity(x_err, y_err):
	if x_err > 0.0:
		vxr = 7.0
	else:
		vxr = 0.0
	if y_err > 0.0:
		vyr = 7.0
	else:
		vyr = 0.0
	return float(vxr), float(vyr)


def calculate_req_yawrate(delta_yaw, velocity):
	max_sw_angle = math.radians(480.0)
	max_wheel_angle = math.radians(34.4)
	steps_per_revolution = 1000.0
	degrees_per_revolution = math.radians(360.0)
	minimum_delay_in_secs = 0.001
	wheel_base = 1.5
	steer_constant = max_wheel_angle / max_sw_angle
	if delta_yaw > max_wheel_angle:
		delta_yaw = max_wheel_angle
	try:
		radius_of_rotation = wheel_base / delta_yaw
	except ZeroDivisionError as straight_line_motion:
		req_yaw_rate = 0.0
	else:
		req_yaw_rate =  float(velocity / radius_of_rotation)
	return req_yaw_rate


def calculate_total_velocity_req(vx, vy, us, yaw_err):
	vt = (vx**2 + vy**2)**0.5
	if us > 0:
		vt = 0
	elif (yaw_err > 10.0) or (yaw_err < -10.0):
		vt = vt / 2
	else:
		vt = vt
	return vt


def calculate_position_req(x, y):
	return float((x**2 + y**2)**0.5)


def calculate_position(x, y):
	return (x**2 + y**2)**0.5


def yaw_to_delta(yaw_to_convert):
	return float(yaw_to_convert)


def yawrate_to_deltad(yawrate_to_convert):
	return yaw_to_delta(yawrate_to_convert)


def calculate_error(req, actual):
	return float(req - actual)