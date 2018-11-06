#! /usr/bin/python

import math
import time
import smbus
import rospy
import roslib
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry


def callback(data):
	global velocity_required
	velocity_required = data.data[0]


def listener():
	rospy.Subscriber('arl_velocity_control', Float32MultiArray, callback)


pwm_out = 0.0
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

while True:
	global velocity_required
	listener()
	velocity_input = velocity_required
	rule1 = ctrl.Rule(velocity['stop']                                                      , pwm['stop']  )
	rule2 = ctrl.Rule(velocity['slow']                                                      , pwm['slow']  )
	rule3 = ctrl.Rule(velocity['medium_slow'] | velocity['medium'] | velocity['medium_fast'], pwm['medium'])
	rule4 = ctrl.Rule(velocity['fast']                                                      , pwm['high']  )
	velocity_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4])
	controlling = ctrl.ControlSystemSimulation(velocity_ctrl)
	controlling.input['velocity_input']
	controlling.compute()
	print controlling.output['pwm']
  	pwm.view(sim=controlling)
  	pwm_out = controlling.output['pwm']
  	pwm_out   = int(pwm_out)
  	auto_mode = int(auto_mode)
  	bus.write_i2c_block_data(drive_address, 0, [auto_mode, pwm_out])
