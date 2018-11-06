#! /usr/bin/python

import math
import time
import rospy
import roslib
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray
from   nav_msgs.msg import Odometry


def talker_velocity(velocity_req):
  pub = rospy.Publisher('arl_velocity_control',Float32MultiArray,queue_size = 10)
  rospy.init_node('talker',anonymous=True)
  r = rospy.Rate(10)
  mat = Float32MultiArray()
  mat.data = [0.0]
  mat.data = [velocity_req]
  pub.publish(mat)
  rospy.loginfo(mat.data)
  r.sleep()


def talker_stepper(yaw_req, yawrate_req):
  pub = rospy.Publisher('arl_stepper_control',Float32MultiArray,queue_size = 10)
  rospy.init_node('talker',anonymous=True)
  r = rospy.Rate(10)
  mat = Float32MultiArray()
  mat.data = [0.0, 0.0]
  mat.data = [yaw_req, yawrate_req]
  pub.publish(mat)
  rospy.loginfo(mat.data)
  r.sleep()


def callback(msg):
    global x_current, y_current, yaw_current, yawrate_current, velocity_current
    x_current       = msg.pose.pose.position.x
    y_current       = msg.pose.pose.position.y
    yaw_current     = math.asin((2 * msg.pose.pose.orientation.w * msg.pose.pose.orientation.z) + (2 * msg.pose.pose.orientation.x * msg.pose.pose.orientation.y))
    vx_current      = msg.twist.twist.linear.x
    vy_current      = msg.twist.twist.linear.y
    yawrate_current = msg.twist.twist.angular.z


def callback1(data):
    global x_required
    x_required = []
    x_required = data.data


def  callback2(data):
    global y_required
    y_required = []
    y_required = data.data


def listener(request):
  teb_request = 'tebmsgrequest'
    if request == teb_request:
      rospy.Subscriber('teb_x', Float64MultiArray, callback1)
      rospy.Subscriber('teb_y', Float64MultiArray, callback2)
    else:
      rospy.Subscriber('arl_sensor_fusion', Odometry, callback)

global x_current, y_current, yaw_current, yawrate_current, velocity_current, x_required, y_required
to_control_drive = [0.0]
to_control_steer = [0.0, 0.0]
pi  = math.radians(180)
pi1 = + pi + math.radians(1)
pi2 = - pi - math.radians(1)
vmax = 4
delta_max = math.radians(34.4)
delta_min = math.radians(- 34.4)
deg1 = math.radians(1)
deg2 = math.radians(10)
deg3 = deg2 / 2
deg4 = math.radians(5)
deg5 = deg4 / 2
l = 1.0
tolerance = 0.1
yawrate_max = vmax * delta_max / l
yawrate_min = vmax * delta_min / l
localization_request_msg = 'localizationmsgrequest'
teb_request_msg          = 'tebmsgrequest'

'''
yawreq                  = ctrl.Antecedent(np.arange(pi2 ,  pi1, deg1), 'yawreq'      )
xcur                    = ctrl.Antecedent(np.arange(-1  , 1001,    1), 'xcur'        )
ycur                    = ctrl.Antecedent(np.arange(-1  , 1001,    1), 'ycur'        )
xdcur                   = ctrl.Antecedent(np.arange(-10 ,   10,  0.1), 'xdcur'       )
ydcur                   = ctrl.Antecedent(np.arange(-10 ,   10,  0.1), 'ydcur'       )
yawdcur                 = ctrl.Antecedent(np.arange(-2.5,  2.5,  0.1), 'yawdcur'     )
'''
position                = ctrl.Antecedent(np.arange(-1  , 1500,    1), 'position'    )
yawerror                = ctrl.Antecedent(np.arange(pi2 ,  pi1, deg1), 'yawerror'    )
velocity                = ctrl.Consequent(np.arange(-0.1,   10,  0.1), 'velocity'    )
yawrate                 = ctrl.Consequent(np.arange(-2.5,  2.5,  0.1), 'yawrate'     )
yaw                     = ctrl.Consequent(np.arange(pi2 ,  pi1, deg1), 'yaw'         )

position['near']        = fuzz.trimf(position.universe, [  0,    0,  250])
position['medium_near'] = fuzz.trimf(position.universe, [  0,  250,  500])
position['medium']      = fuzz.trimf(position.universe, [250,  500,  750])
position['medium_far']  = fuzz.trimf(position.universe, [500,  750, 1000])
position['far']         = fuzz.trimf(position.universe, [750, 1000, 1000])

yawerror['turn_left']   = fuzz.trimf(yawerror.universe, [  -pi,   -pi, -deg2])
yawerror['slide_left']  = fuzz.trimf(yawerror.universe, [-deg2, -deg3,     0])
yawerror['do_nothing']  = fuzz.trimf(yawerror.universe, [-deg1,     0,  deg1])
yawerror['slide_right'] = fuzz.trimf(yawerror.universe, [    0,  deg3,  deg2])
yawerror['turn_right']  = fuzz.trimf(yawerror.universe, [ deg2,    pi,    pi])

yaw['left_steer']       = fuzz.trimf(yaw.universe, [delta_min, delta_min,     -deg4])
yaw['slide_left']       = fuzz.trimf(yaw.universe, [    -deg4,     -deg5,         0])
yaw['straight']         = fuzz.trimf(yaw.universe, [    -deg1,         0,      deg1])
yaw['slide_right']      = fuzz.trimf(yaw.universe, [        0,      deg5,      deg4])
yaw['right_steer']      = fuzz.trimf(yaw.universe, [     deg4, delta_max, delta_max])

yawrate['left']         = fuzz.trimf(yawrate.universe, [ -2.4,   -2.4, -0.01])
yawrate['mleft']        = fuzz.trimf(yawrate.universe, [-0.01, -0.005,     0])
yawrate['middle']       = fuzz.trimf(yawrate.universe, [-0.01,      0,  0.01])
yawrate['mright']       = fuzz.trimf(yawrate.universe, [    0,  0.005,  0.01])
yawrate['right']        = fuzz.trimf(yawrate.universe, [ 0.01,    2.4,   2.4])

velocity['low']         = fuzz.trimf(velocity.universe, [0, 0, 3])
velocity['medium_low']  = fuzz.trimf(velocity.universe, [0, 3, 5])
velocity['medium']      = fuzz.trimf(velocity.universe, [3, 5, 7])
velocity['medium_high'] = fuzz.trimf(velocity.universe, [5, 7, 9])
velocity['high']        = fuzz.trimf(velocity.universe, [7, 9, 9])

velocity.view()
yawrate.view()
yawerror.view()
position.view()
yaw.view()

while True:
  listener(teb_request_msg)
  number_of_plan_points = x_required.__len__()
  if number_of_plan_points > 0:
    break

i = 0

while not rospy.is_shutdown():

	while i <= number_of_plan_points:

  		listener(localization_request_msg)

	  	xfreq_input    = x_required[-1]
  		yfreq_input    = y_required[-1]
	  	xreq_input     = x_required[i]
 		yreq_input     = y_required[i]
  		xcur_input     = x_current
 		ycur_input     = y_current
  		yawcur_input   = yaw_current
  		yawdcur_input  = yawrate_current
  		position_input = ((xfreq_input**2 + yfreq_input**2)**0.5) - ((xcur_input**2 + ycur_input**2)**0.5)
  		yawreq_input   = math.atan2(yreq_input, xreq_input)
  		yawerror_input = yawreq_input - yawcur_input
  		velocity_input = (vx_current**2 + vy_current**2)**0.5

	  	rule01 = ctrl.Rule(position['near']                                                    , velocity['low'])
  		rule02 = ctrl.Rule(position['medium_near']                                             , velocity['medium_low'])
  		rule03 = ctrl.Rule(position['medium'] | yawerror['turn_right'] | yawerror['turn_left'] , velocity['medium'])
  		rule04 = ctrl.Rule(position['medium_far']                                              , velocity['medium_high'])
  		rule05 = ctrl.Rule(position['far']                                                     , velocity['high'])
  		rule06 = ctrl.Rule(yawerror['slide_left']                                              , yawrate['mleft'])
  		rule07 = ctrl.Rule(yawerror['slide_right']                                             , yawrate['mright'])
  		rule08 = ctrl.Rule(yawerror['turn_left']                                               , yawrate['left'])
  		rule09 = ctrl.Rule(yawerror['turn_right']                                              , yawrate['right'])
  		rule10 = ctrl.Rule(yawerror['do_nothing']                                              , yawrate['middle'])
  		rule11 = ctrl.Rule(yawerror['slide_right']                                             , yaw['slide_right'])
  		rule12 = ctrl.Rule(yawerror['slide_left']                                              , yaw['slide_left'])
  		rule13 = ctrl.Rule(yawerror['turn_left']                                               , yaw['left_steer'])
  		rule14 = ctrl.Rule(yawerror['turn_right']                                              , yaw['right_steer'])
  		rule15 = ctrl.Rule(yawerror['do_nothing']                                              , yaw['straight'])

	  	rule1.view()

  		velocity_ctrl = ctrl.ControlSystem([rule01, rule02, rule03, rule04, rule05])
  		yawrate_ctrl  = ctrl.ControlSystem([rule06, rule07, rule08, rule09, rule10])
  		yaw_ctrl      = ctrl.ControlSystem([rule11, rule12, rule13, rule14, rule15])

  		controlling1 = ctrl.ControlSystemSimulation(velocity_ctrl)
  		controlling2 = ctrl.ControlSystemSimulation(yawrate_ctrl)
  		controlling3 = ctrl.ControlSystemSimulation(yaw_ctrl)

 	 	controlling1.input['position'] = position_input
  		controlling1.input['yawerror'] = yawerror_input
  		controlling2.input['yawerror'] = yawerror_input
  		controlling3.input['yawerror'] = yawerror_input

  		controlling1.compute()
  		controlling2.compute()
  		controlling3.compute()

	  	print controlling1.output['velocity']
  		velocity.view(sim=controlling1)
  		print controlling2.output['yawrate']
  		yawrate.view(sim=controlling2)
  		print controlling3.output['yaw']
  		yaw.view(sim=controlling3)
    		if ((xcur_input + tolerance) == xreq_input) or ((xcur_input - tolerance) == xreq_input):
      			if ((ycur_input + tolerance) == yreq_input) or ((ycur_input - tolerance) == yreq_input):
        			i += 1

		to_control_drive = [controlling1.output['velocity']]
		to_control_steer = [controlling3.output['yaw'], controlling2.output['yawrate']]

	    	talker_velocity(to_control_drive)
    		talker_stepper(to_control_steer)

def pause():
    programPause = raw_input("Press the <ENTER> key to continue...")

print("Check the output curves....")
pause()
