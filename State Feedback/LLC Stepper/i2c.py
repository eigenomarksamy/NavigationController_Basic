#! /usr/bin/python

import math
import os
import smbus
import time
from datetime import datetime


def write_to(system, system_parameters):
	bus = smbus.SMBus(1)
	drive_address = 0x05
	steer_address = 0x04
	brake_address = 0x03
	drive_index = 0
	steer_index = 1
	brake_index = 2
	if system == drive_index:
		address = drive_address
		drive_state_to_send = system_parameters[0]
		pwm_to_send = system_parameters[1]
		bus.write_i2c_block_data(address, 0, [drive_state_to_send, pwm_to_send])
	elif system == steer_index:
		steps_byte = [11] * 0
		address = steer_address
		steer_state_to_send = int(system_parameters[0])
		steer_direction_to_send = int(system_parameters[1])
		steer_delay_to_send = int(math.ceil(system_parameters[3] / 10))
		steer_steps = int(system_parameters[2])
		number_of_bytes = int(math.ceil(steer_steps / 255))
		max_value = 255 * (number_of_bytes)
		for i in range(n):
			steps_byte[i] = 255
		steps_byte[n] = steer_steps - max_value
		bus.write_i2c_block_data(address, 0, [steer_state_to_send, steer_direction_to_send, steer_delay_to_send, steps_byte[0], steps_byte[1], steps_byte[2], steps_byte[3], steps_byte[4], steps_byte[5], steps_byte[6], steps_byte[7], steps_byte[8], steps_byte[9], steps_byte[10]])
	else:
		address = brake_address
