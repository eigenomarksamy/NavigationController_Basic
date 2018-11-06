#! /usr/bin/python

import publisher
import estimator
import time
import rospy
from std_msgs.msg import Float32MultiArray
from   nav_msgs.msg import Odometry


def callback(data):
    global localization
    global current_index, x_index, y_index, yaw_index
    localization[x_index][current_index]   = data.data[0]
    localization[y_index][current_index]   = data.data[1]
    localization[yaw_index][current_index] = data.data[2]


def callback1(data):
    global localization
    global yawrate_index, current_index
    localization[yawrate_index][current_index] = data.data[9]


def callback2(msg):
    global localization
    localization[xvelocity_index][current_index] = msg.twist.twist.linear.x
    localization[yvelocity_index][current_index] = msg.twist.twist.linear.y


def callback3(data):
    global localization, stamp_index, current_index
    localization[stamp_index][current_index] = data.data[0]


def callback4(data):
    global localization, final_position
    global x_index, req_index, y_index, yaw_index
    localization[x_index][req_index]   = data.data[0]
    localization[y_index][req_index]   = data.data[1]
    localization[yaw_index][req_index] = data.data[2]
    final_position[x_index]            = data.data[3]
    final_position[y_index]            = data.data[4]


def callback5(data):
    global x_required
    x_required = data.data


def callback6(data):
    global y_required
    y_required = data.data


def listener_localization():
    rospy.Subscriber('localization_Fusion', Float32MultiArray, callback)
    rospy.Subscriber('imu_data', Float32MultiArray, callback1)
    rospy.Subscriber('teb', Float32MultiArray, callback4)
    rospy.Subscriber('odom', Odometry, callback2)
    rospy.Subscriber('stamp', Float32MultiArray, callback3)
    main()
    rospy.spin()


def listener_planner():
    rospy.Subscriber('teb_x', Float64MultiArray, callback5)
    rospy.Subscriber('teb_y', Float64MultiArray, callback6)


def previous_state_assignment():
    global localization, final_position, us, current_index, previous_index, req_index, req_previous_index, x_index, y_index, yaw_index, xvelocity_index, yvelocity_index, yawrate_index, stamp_index, number_of_plan_points, planning_counter, err, command, velocity_index, position_index, delta_index, deltad_index, drive_system, steer_system, brake_system, to_control_drive, to_control_steer, to_control_brake, tvelocity_index
    localization[x_index][previous_index]             = localization[x_index][current_index]
    localization[y_index][previous_index]             = localization[y_index][current_index]
    localization[yaw_index][previous_index]           = localization[yaw_index][current_index]
    localization[xvelocity_index][previous_index]     = localization[xvelocity_index][current_index]
    localization[yvelocity_index][previous_index]     = localization[yvelocity_index][current_index]
    localization[yawrate_index][previous_index]       = localization[yawrate_index][current_index]
    localization[tvelocity_index][previous_index]     = localization[tvelocity_index][current_index]
    localization[position_index][previous_index]      = localization[position_index][current_index]
    localization[stamp_index][previous_index]         = localization[stamp_index][current_index]
    localization[x_index][req_previous_index]         = localization[x_index][req_index]
    localization[y_index][req_previous_index]         = localization[y_index][req_index]
    localization[yaw_index][req_previous_index]       = localization[yaw_index][req_index]
    localization[xvelocity_index][req_previous_index] = localization[xvelocity_index][req_index]
    localization[yvelocity_index][req_previous_index] = localization[yvelocity_index][req_index]
    localization[yawrate_index][req_previous_index]   = localization[yawrate_index][req_index]
    localization[tvelocity_index][req_previous_index] = localization[tvelocity_index][req_index]
    localization[position_index][req_previous_index]  = localization[position_index][req_index]


def get_planning_points():
    global x_required, y_required, number_of_plan_points_x, number_of_plan_points_y, counter_planner, number_of_iterations
    while counter_planner < number_of_iterations:
            listener_planner()
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
    global localization, x_index, y_index, req_index, planning_counter, x_required, y_required
    localization[x_index][req_index] = x_required[planning_counter]
    localization[y_index][req_index] = y_required[planning_counter]


def main():
    global localization, final_position, us, current_index, previous_index, req_index, req_previous_index, x_index, y_index, yaw_index, xvelocity_index, yvelocity_index, yawrate_index, stamp_index, number_of_plan_points, planning_counter, err, command, velocity_index, position_index, delta_index, deltad_index, drive_system, steer_system, brake_system, to_control_drive, to_control_steer, to_control_brake, tvelocity_index
# Error calculation
    if localization[stamp_index][current_index] > localization[stamp_index][previous_index]:
        print localization[stamp_index]
        if ((xcur_input + tolerance) == xreq_input) or ((xcur_input - tolerance) == xreq_input):
            if ((ycur_input + tolerance) == yreq_input) or ((ycur_input - tolerance) == yreq_input):
                planning_point_index += 1
        if planning_counter <= number_of_plan_points:
            update_values()
            localization[yaw_index][req_index] = estimator.calculate_req_yaw(localization[x_index][req_index], localization[y_index][req_index])
            err[x_index] = estimator.calculate_error(final_position[x_index], localization[x_index][current_index])
            err[y_index] = estimator.calculate_error(final_position[y_index], localization[y_index][current_index])
            err[yaw_index] = estimator.calculate_error(localization[yaw_index][req_index], localization[yaw_index][current_index])
            err[yawrate_index] = estimator.calculate_error(localization[yawrate_index][req_index], localization[yawrate_index][current_index])
# Estimation of the setpoints
            localization[xvelocity_index][req_index], localization[yvelocity_index][req_index] = estimator.calculate_req_velocity(err[x_index], err[y_index])
            localization[yawrate_index][req_index] = estimator.calculate_req_yawrate(err[yaw_index], localization[tvelocity_index][current_index])
            localization[tvelocity_index][req_index] = estimator.calculate_total_velocity_req(localization[xvelocity_index][req_index], localization[yvelocity_index][req_index], us[current_index], err[yaw_index])
            localization[position_index][req_index] = estimator.calculate_position_req(err[x_index], err[y_index])
            localization[position_index][current_index] = estimator.calculate_position(localization[x_index][current_index], localization[y_index][current_index])
# Assignment of commands
            command[velocity_index] = localization[tvelocity_index][req_index]
            command[delta_index]    = estimator.yaw_to_delta(err[yaw_index])
            command[deltad_index]   = estimator.yawrate_to_deltad(err[yawrate_index])
# Creating Lists
            drive_system = [command[velocity_index]]
            steer_system = [command[delta_index], command[deltad_index]]
            brake_system = [localization[position_index][current_index], localization[position_index][req_index], us[current_index]]
# Creating control actions lists
            to_control_drive = drive_system
            to_control_steer = steer_system
            to_control_brake = brake_system
# Write to Multimaster
            if not rospy.is_shutdown():
                publisher.talker_velocity(to_control_drive)
                publisher.talker_stepper(to_control_steer, to_control_brake)
            else:
                print "Cannot publish data due to loss in communication."
        else:
# Else of planner
            print "Trajectory"
    else:
        print "Outdated loco"
# Print values
	print "Localization is: "
	print localization
	print "Final position is: "
	print final_position
	print "Ultrasonic is: "
	print us
# Assignment of previous states
    previous_state_assignment()


if __name__ =='__main__':
# Initialization
    global localization, final_position, us, current_index, previous_index, req_index, req_previous_index, x_index, y_index, yaw_index, xvelocity_index, yvelocity_index, yawrate_index, stamp_index, number_of_plan_points, planning_counter, err, command, velocity_index, position_index, delta_index, deltad_index, drive_system, steer_system, brake_system, to_control_drive, to_control_steer, to_control_brake, tvelocity_index
    current_index      = 0
    previous_index     = 1
    req_index          = 2
    req_previous_index = 3
    xfinal_index       = 0
    yfinal_index       = 1
    x_index            = 0
    y_index            = 1
    yaw_index          = 2
    xvelocity_index    = 3
    yvelocity_index    = 4
    yawrate_index      = 5
    tvelocity_index    = 6
    position_index     = 7
    velocity_index     = 0
    delta_index        = 1
    deltad_index       = 2
    stamp_index        = 8
    localization       = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
    err                = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    command            = [0.0, 0.0, 0.0]
    final_position     = [0.0, 0.0]
    us                 = [0, 0]
    ts                 = 1.0
    x_required         = []
    y_required         = []
    drive_system       = []
    steer_system       = []
    brake_system       = []
    to_control_drive   = []
    to_control_steer   = []
    to_control_brake   = []
    planning_counter   = 0
    number_of_plan_points = 0
    get_planning_points()
    check_planning_points()
    while True:
        listener_localization()
