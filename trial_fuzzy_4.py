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
from nav_msgs.msg import Odometry
#from std_msgs.header import uint32


def pause():
	programPause = raw_input("Press the <ENTER> key to continue...")


#def talker_velocity(velocity_req):
#	pub = rospy.Publisher('arl_velocity_control',Float32MultiArray,queue_size = 10)
# 	rospy.init_node('talker',anonymous=True)
#  	r = rospy.Rate(10)
#  	mat = Float32MultiArray()
#  	mat.data = [0.0]
#  	mat.data = [velocity_req]
#  	pub.publish(mat)
#  	rospy.loginfo(mat.data)
#  	r.sleep()


def talker_llc(talker_stamp, drive_control_actions, steer_control_actions):
	yaw_control_action = steer_control_actions[0]
  yawrate_control_action = steer_control_actions[1]
  velocity_control_action = drive_control_actions[0]
  pub = rospy.Publisher('llc_control', Float32MultiArray, queue_size = 10)
  	rospy.init_node('talker',anonymous=True)
  	r = rospy.Rate(10)
  	mat = Float32MultiArray()
  	mat.data = [0, 0.0, 0.0, 0.0]
  	mat.data = [talker_stamp, velocity_control_action, yaw_control_action, yawrate_control_action]
  	pub.publish(mat)
  	rospy.loginfo(mat.data)
  	r.sleep()


def callback(msg):
    	global x_current, y_current, yaw_current, yawrate_current, vx_current, vy_current, localization_stamp, current_index
    	x_current       = msg.pose.pose.position.x
    	y_current       = msg.pose.pose.position.y
    	yaw_current     = math.asin((2 * msg.pose.pose.orientation.w * msg.pose.pose.orientation.z) + (2 * msg.pose.pose.orientation.x * msg.pose.pose.orientation.y))
    	vx_current      = msg.twist.twist.linear.x
    	vy_current      = msg.twist.twist.linear.y
    	yawrate_current = msg.twist.twist.angular.z


#def callback3(msg):
#    	localization_stamp[current_index] = msg.header.header.seq


def callback1(data):
	global x_required
  	x_required = data.data


def callback2(data):
  	global y_required
  	y_required = data.data


def callback4(data):
	global localization_stamp, current_index, x_current, y_current, vx_current, vy_current, yaw_current, yawrate_current, localization_stamp_id
  	localization_stamp_id.append(int(data.data[0]))
    	print "Localization Stamp ID: ", localization_stamp_id
	localization_stamp[current_index] = int(data.data[0])
	x_current       = data.data[1]
	y_current       = data.data[2]
	yaw_current     = data.data[3]
  	vx_current      = data.data[4]
  	vy_current      = data.data[5]
  	yawrate_current = data.data[6]
    	main()


def listener(request):
  	rospy.init_node('arl_navigation_control')
  	teb_request = 'tebmsgrequest'
  	if request == teb_request:
  		rospy.Subscriber('teb_x', Float64MultiArray, callback1)
  		rospy.Subscriber('teb_y', Float64MultiArray, callback2)
  	else:
   		rospy.Subscriber('arl_sensor_fusion', Odometry, callback)
# 		rospy.Subscriber('arl_sensor_fusion', uint32, callback3)
#  		print "XXKEOF"
#   		rospy.Subscriber('sensor_fusion', Float64MultiArray, callback4)
      		rospy.spin()

def fuzzy(position_input_fuz, yawerror_input_fuz):
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
  	yawrate_max = vmax * delta_max / l
  	yawrate_min = vmax * delta_min / l
  	position                = ctrl.Antecedent(np.arange(-1501, 1501,    1), 'position'    )
  	yawerror                = ctrl.Antecedent(np.arange(pi2  ,  pi1, deg1), 'yawerror'    )
  	velocity                = ctrl.Consequent(np.arange(-0.1 ,   10,  0.1), 'velocity'    )
  	yawrate                 = ctrl.Consequent(np.arange(-2.5 ,  2.5,  0.1), 'yawrate'     )
  	yaw                     = ctrl.Consequent(np.arange(pi2  ,  pi1, deg1), 'yaw'         )

	position['near_n']        = fuzz.trimf(position.universe, [   -300,    0,  0])
    	position['medium_near_n'] = fuzz.trimf(position.universe, [   -600, -300,  0])
    	position['medium_n']      = fuzz.trimf(position.universe, [-900, -600,  -300])
    	position['medium_far_n']  = fuzz.trimf(position.universe, [-1200, -900, -600])
    	position['far_n']         = fuzz.trimf(position.universe, [-1500,-1200, -900])
  	position['near_p']        = fuzz.trimf(position.universe, [  0,     0,   300])
  	position['medium_near_p'] = fuzz.trimf(position.universe, [  0,   300,   600])
  	position['medium_p']      = fuzz.trimf(position.universe, [300,   600,   900])
  	position['medium_far_p']  = fuzz.trimf(position.universe, [600,   900,  1200])
  	position['far_p']         = fuzz.trimf(position.universe, [900,  1200,  1500])

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

#  velocity.view()
#  yawrate.view()
#  yawerror.view()
#  position.view()
#  yaw.view()
  	rule01 = ctrl.Rule(position['near_n'] | position['near_p']                             , velocity['low'])
  	rule02 = ctrl.Rule(position['medium_near_n'] | position['medium_near_p']               , velocity['medium_low'])
  	rule03 = ctrl.Rule(position['medium_n'] | position['medium_n'] | yawerror['turn_right'] | yawerror['turn_left'] , velocity['medium'])
  	rule04 = ctrl.Rule(position['medium_far_n'] | position['medium_far_p']                                             , velocity['medium_high'])
  	rule05 = ctrl.Rule(position['far_n'] | position['far_p']                                                     , velocity['high'])
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

  	rule01.view()

  	velocity_ctrl_fuz = ctrl.ControlSystem([rule01, rule02, rule03, rule04, rule05])
  	yawrate_ctrl_fuz  = ctrl.ControlSystem([rule06, rule07, rule08, rule09, rule10])
  	yaw_ctrl_fuz      = ctrl.ControlSystem([rule11, rule12, rule13, rule14, rule15])

  	controlling1_fuz = ctrl.ControlSystemSimulation(velocity_ctrl_fuz)
  	controlling2_fuz = ctrl.ControlSystemSimulation(yawrate_ctrl_fuz)
  	controlling3_fuz = ctrl.ControlSystemSimulation(yaw_ctrl_fuz)

  	controlling1_fuz.input['position'] = position_input_fuz
  	controlling1_fuz.input['yawerror'] = yawerror_input_fuz
  	controlling2_fuz.input['yawerror'] = yawerror_input_fuz
  	controlling3_fuz.input['yawerror'] = yawerror_input_fuz

  	controlling1_fuz.compute()
  	controlling2_fuz.compute()
  	controlling3_fuz.compute()

#  	print controlling1.output['velocity']
#  	print controlling2.output['yawrate']
#  	print controlling3.output['yaw']
  	return controlling1_fuz.output['velocity'], controlling3_fuz.output['yaw'], controlling2_fuz.output['yawrate']


def get_planning_points():
  	global x_required, y_required, number_of_plan_points_x, number_of_plan_points_y, counter_planner, teb_request_msg, number_of_iterations
  	while counter_planner < number_of_iterations:
    		listener(teb_request_msg)
    		number_of_plan_points_x = x_required.__len__()
    		number_of_plan_points_y = y_required.__len__()
    		counter_planner += 1


def check_planning_points():
  	global number_of_plan_points_x, number_of_plan_points_y, number_of_plan_points
  	if number_of_plan_points_x == number_of_plan_points_y:
    		number_of_plan_points = number_of_plan_points_x
    		if number_of_plan_points > 0:
      			print "Planning points: ", number_of_plan_points
    		else:
      			print "Reconnecting to planner."
  	else:
    		print "Not sufficient amount of data"


def update_values():
  	global xreq_input, yreq_input, xcur_input, ycur_input, yaw_current, yawdcur_input, position_input, yawreq_input, yawerror_input, velocity_input, x_required, y_required, planning_point_index, x_current, y_current, yaw_current, yawrate_current, xfreq_input, yfreq_input, vx_current, vy_current
  	xreq_input     = x_required[planning_point_index]
  	yreq_input     = y_required[planning_point_index]
  	xcur_input     = x_current
  	ycur_input     = y_current
  	yawcur_input   = yaw_current
  	yawdcur_input  = yawrate_current
  	position_input = ((xfreq_input**2 + yfreq_input**2)**0.5) - ((xcur_input**2 + ycur_input**2)**0.5)
  	yawreq_input   = math.atan2(yreq_input, xreq_input)
  	yawerror_input = yawreq_input - yawcur_input
  	velocity_input = (vx_current**2 + vy_current**2)**0.5


def main():
  	global x_current, y_current, yaw_current, yawrate_current, vx_current, vy_current, x_required, y_required, localization_stamp, current_index, previous_index, number_of_plan_points_x, number_of_plan_points_y, number_of_plan_points, counter_planner, localization_request_msg, teb_request_msg, number_of_iterations, tolerance, localization_stamp_id, planning_point_index, yreq_input, ycur_input, xreq_input, xcur_input, localization_id, yfreq_input, xfreq_input, velocity_input, yawerror_input, yawreq_input, position_input, yawdcur_input
#  	listener(localization_request_msg)
#  	localization_id = localization_stamp[current_index] - localization_initial_index
  	if localization_stamp[current_index] > localization_stamp[previous_index]:
    		print "loco: ", localization_stamp[current_index], localization_stamp[previous_index]
        if ((xcur_input + tolerance) == xreq_input) or ((xcur_input - tolerance) == xreq_input):
              if ((ycur_input + tolerance) == yreq_input) or ((ycur_input - tolerance) == yreq_input):
                  planning_point_index += 1
		    if planning_point_index <= number_of_plan_points:
            print "Planning point index: ", planning_point_index, "/", number_of_plan_points
      			update_values()
      			to_control_drive[0], to_control_steer[0], to_control_steer[1] = fuzzy(position_input, yawerror_input)
			print "Drive: ", to_control_drive
			print "Steer: ", to_control_steer
    			talker_llc(planning_point_index, to_control_drive, to_control_steer)
#			listener(localization_request_msg)
		    else:
      			print "Achieved Trajectory."
  	else:
    		print "Outdated localization."
  	localization_stamp[previous_index] = localization_stamp[current_index]


if __name__ == '__main__':
  	global x_current, y_current, yaw_current, yawrate_current, vx_current, vy_current, x_required, y_required, localization_stamp, current_index, previous_index, number_of_plan_points_x, number_of_plan_points_y, number_of_plan_points, counter_planner, localization_request_msg, teb_request_msg, number_of_iterations, tolerance, localization_stamp_id, planning_point_index, yreq_input, ycur_input, xreq_input, xcur_input, localization_id, yfreq_input, xfreq_input, velocity_input, yawerror_input, yawreq_input, position_input, yawdcur_input
  	localization_stamp_id = []
  	tolerance = 0.1
  	counter_planner = 0
  	planning_point_index = 0
  	number_of_plan_points = 0
  	number_of_iterations = 10000
  	localization_stamp = [0]*2
  	current_index = 0
  	previous_index = 1
  	x_current = 0.0
  	y_current = 0.0
  	yaw_current = 0.0
  	vx_current = 0.0
  	vy_current = 0.0
  	yawrate_current = 0.0
  	x_required = []
  	y_required = []
  	to_control_drive = [0.0]
  	to_control_steer = [0.0, 0.0]
  	localization_request_msg = 'localizationmsgrequest'
  	teb_request_msg          = 'tebmsgrequest'
  	get_planning_points()
  	check_planning_points()

  	xfreq_input    = x_required[-1]
  	yfreq_input    = y_required[-1]

	print "X-Final", xfreq_input
	print "Y-Final", yfreq_input

#  	localization_initial_id = localization_stamp_id[0]
#while True:
#while not rospy.is_shutdown():
  	while True:
  		listener(localization_request_msg)
#	   print "Localization stamp: ", localization_id

#  	if localization_stamp[current_index] > localization_stamp[previous_index]:
#		  print "X-req: ", xreq_input
#		  print "Y-req: ", yreq_input
#		  print "X-cur: ", xcur_input
#		  print "Y-cur: ", ycur_input
#		  print "A-cur: ", yawcur_input
#		  print "Ad-cur: ", yawdcur_input
#		  print "P-err: ", position_input
#		  print "A-req: ", yawreq_input
#		  print "A-err: ", yawerror_input
#		  print "V-cur: ", velocity_input

#     to_control_drive = [controlling1.output['velocity']]
#     to_control_steer = [controlling3.output['yaw'], controlling2.output['yawrate']]

#     talker_velocity(to_control_drive)
#     talker_stepper(to_control_steer)


#          	rospy.spin()
#    print "--------"

#        rospy.spin()
#  else:
#	  print "Achieved Trajectory."
#else:
#	"Disconnected from multimaster."

  	print("Check the output curves....")
  	pause()
