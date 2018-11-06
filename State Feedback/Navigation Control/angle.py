#! /usr/bin/python
import math
# delta = ks * theta
initial_delta = math.radians(0.0)
initial_theta = math.radians(0.0)
required_delta = math.radians(90.0)
current_delta = initial_delta
current_theta = initial_theta
ks = math.radians(34.0) / math.radians(480.0)
i = 0
required_delta = required_delta - current_delta
required_theta = required_delta / ks
max_theta = math.radians(480.0) - current_theta
while (required_delta > current_delta) or (required_delta < current_delta):
	print required_delta
	print current_delta
	print required_theta
	print current_theta
	print max_theta
	print '-------------------------'
	required_delta = required_delta - current_delta
	required_theta = required_delta / ks
	max_theta = math.radians(480.0) - current_theta
	if required_theta > (max_theta):
		required_theta = max_theta
	current_theta = required_theta
	current_delta = current_theta * ks
	i += 1
	print required_delta
	print current_delta
	print required_theta
	print current_theta
	print max_theta
	print '************************'
print i