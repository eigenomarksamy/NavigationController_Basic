#! /usr/bin/python

import math

pi = math.radians(180)
pi1 = pi + math.radians(1)
pi2 = pi - math.radians(1)
pi3 = - pi - math.radians(1)
pi4 = - pi2
print math.tan(pi)
print -pi
print pi1
print -pi2
print pi3
print pi4
vmax = + 4
delta_max = math.radians(34.4)
delta_min = math.radians(- 34.4)
l = 1.1
yawrate_max = vmax * delta_max / l
yawrate_min = vmax * delta_min / l
print -yawrate_min
print -yawrate_max
print math.degrees(pi1 - pi)