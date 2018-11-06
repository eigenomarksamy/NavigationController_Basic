#! /usr/bin/python

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

position_input = input("Enter how far from the final goal: ")
yawerror_input = input("Enter the error in yaw angle: ")

position           = ctrl.Antecedent(np.arange(0, 100, 1), 'position')
yawerror           = ctrl.Antecedent(np.arange(-180, 180, 1), 'yawerror')
velocity           = ctrl.Consequent(np.arange(0, 11, 1), 'velocity')
yawrate            = ctrl.Consequent(np.arange(0, 7, 1), 'yawrate')

yawrate['low']          = fuzz.trimf(yawrate.universe, [0, 0, 3])
yawrate['medium']       = fuzz.trimf(yawrate.universe, [0, 3, 6])
yawrate['high']         = fuzz.trimf(yawrate.universe, [3, 6, 6])

velocity['low']         = fuzz.trimf(velocity.universe, [0, 0, 3])
velocity['medium_low']  = fuzz.trimf(velocity.universe, [0, 3, 5])
velocity['medium']      = fuzz.trimf(velocity.universe, [3, 5, 7])
velocity['medium_high'] = fuzz.trimf(velocity.universe, [5, 7, 10])
velocity['high']        = fuzz.trimf(velocity.universe, [7, 10, 10])

position['near']        = fuzz.trimf(position.universe, [0, 0, 25])
position['medium_near'] = fuzz.trimf(position.universe, [0, 25, 50])
position['medium']      = fuzz.trimf(position.universe, [25, 50, 75])
position['medium_far']  = fuzz.trimf(position.universe, [50, 75, 100])
position['far']         = fuzz.trimf(position.universe, [75, 100, 100])

yawerror['turn_left']   = fuzz.trimf(yawerror.universe, [-179, -179, -10])
yawerror['slide_left']  = fuzz.trimf(yawerror.universe, [-10, -10, 0])
yawerror['slide_right'] = fuzz.trimf(yawerror.universe, [0, 10, 10])
yawerror['turn_right']  = fuzz.trimf(yawerror.universe, [10, 179, 179])
yawerror['do_nothing']  = fuzz.trimf(yawerror.universe, [-1, 0, 1])

velocity.view()
yawrate.view()
yawerror.view()
position.view()

rule1 = ctrl.Rule(position['near'],                                                     velocity['low'])
rule2 = ctrl.Rule(position['medium_near'],                                              velocity['medium_low'])
rule3 = ctrl.Rule(position['medium'] | yawerror['turn_right'] | yawerror['turn_left'],  velocity['medium'])
rule4 = ctrl.Rule(position['medium_far'],                                               velocity['medium_high'])
rule5 = ctrl.Rule(position['far'],                                                      velocity['high'])
rule6 = ctrl.Rule(yawerror['slide_left'] | yawerror['slide_right'],                     yawrate['medium'])
rule7 = ctrl.Rule(yawerror['turn_left'] | yawerror['turn_right'],                       yawrate['high'])
rule8 = ctrl.Rule(yawerror['do_nothing'],                                               yawrate['low'])

velocity_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
yawrate_ctrl  = ctrl.ControlSystem([rule6, rule7, rule8])

controlling1 = ctrl.ControlSystemSimulation(velocity_ctrl)
controlling2 = ctrl.ControlSystemSimulation(yawrate_ctrl)

controlling1.input['position'] = position_input
controlling1.input['yawerror'] = yawerror_input
controlling2.input['yawerror'] = yawerror_input

controlling1.compute()
controlling2.compute()

print controlling1.output['velocity']
velocity.view(sim=controlling1)
print controlling2.output['yawrate']
yawrate.view(sim=controlling2)

def pause():
    programPause = raw_input("Press the <ENTER> key to continue...")

print("Check the output curves....")
pause()