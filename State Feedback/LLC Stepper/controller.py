#! /usr/bin/python

import stepper


def steer_control(steer_list):
    delta_yaw = steer_list[0]
    wheel_rate = steer_list[1]
    wheel_position = steer_list[2]
    steer = 1
    max_sw_angle = math.radians(480.0)
    max_tires_angle = math.radians(34.4)
    k_steer = max_tires_angle / max_sw_angle
    sliding_threshold = math.radians(10.0)
    null = 0
    turning = 1
    sliding = 2
    err_in_heading = delta_yaw
    if err_in_heading > 0.0:
        if err_in_heading <= sliding_threshold:
            state = sliding
        else:
            state =turning
    elif err_in_heading < 0.0:
        if err_in_heading >= (-1 * sliding_threshold):
            state = sliding
        else:
            state = turning
    else:
        state = null
    sw_position_req = err_in_heading * k_steer
    direction = stepper.calc_direction(steer, err_in_heading)
    steps = stepper.calc_steps(steer, err_in_heading, wheel_position)
    delay = stepper.calc_delay(steer, steps, wheel_rate)
    return state, direction, steps, delay


def brake_control(position, final_position, interrupt_action, pedal_rate):
    brake_system_index = 2
    null = 0
    stop_factor = 1
    emergency_braking = 2
    normal_braking = 1
    emergency_threshold = 3                                                                             # threshold in meters
    braking = 1
    counter_braking = 0
    pedal_to_steps = 1
    max_steps = 0
    min_delay = 0
    if interrupt_action == null:
        err = stepper.calc_error(brake_system_index, position, final_position)
        err = err * stop_factor                                                                         # convert error to stopping distance using some constants
        direction = stepper.calc_direction(brake_system_index, err, 0, pedal)
        steps = stepper.calc_steps(brake_system_index, err, pedal)
        delay = stepper.calc_delay(brake_system_index, steps, pedal_rate)
        if (err > emergency_threshold):                                                                 # emergency threshold to be set related to the err unit
            braking_state = emergency_braking
        elif (err < emergency_threshold):
            braking_state = normal_braking
        else:
            state = null
    else:
        direction = braking                                                                             # brake or release
        pedal_steps = pedal * pedal_to_steps                                                            # convert pedal position to steps
        steps = max_steps - pedal_steps                                                                 # write the max available steps
        delay = min_delay                                                                               # minimum delay
        braking_state = emergency_braking                                                               # emergency state
    braking_state = 0
    return braking_state, direction, steps, delay


def drive_control(steer_state, brake_state, vel, req_vel, pos, req_pos, final_position):
    steer_state = 1
    brake_state = 2
    no_steering = 0
    turning = 1
    no_braking = 0
    normal_braking = 1
    emergency_braking = 2
    normal_driving = 1
    no_driving = 0
    slow_driving = 2
    constant_pwm = 35
    null = 0
    if steer_state == no_steering:
        if brake_state == no_braking:
            pwm = constant_pwm
            state = normal_driving
        elif brake_state == normal_braking:
            pwm = null
            state = no_driving
        else:
            pwm = null
            state = no_driving
    elif steer_state == turning:
        if brake_state == no_braking:
            pwm = constant_pwm
            state = slow_driving
        elif brake_state == normal_braking:
            pwm = null
            state = no_driving
        else:
            pwm = null
            state = no_driving
    else:
        if brake_state == no_braking:
            pwm = constant_pwm
            state = slow_driving
        elif brake_state == normal_braking:
            pwm = null
            state = no_driving
        else:
            pwm = null
            state = no_driving
    return state, pwm
