#! /usr/bin/python
"""
==========================================
Fuzzy Control Systems: The Tipping Problem
==========================================

The 'tipping problem' is commonly used to illustrate the power of fuzzy logic
principles to generate complex behavior from a compact, intuitive set of
expert rules.

If you're new to the world of fuzzy control systems, you might want
to check out the `Fuzzy Control Primer
<../userguide/fuzzy_control_primer.html>`_
before reading through this worked example.

The Tipping Problem
-------------------

Let's create a fuzzy control system which models how you might choose to tip
at a restaurant.  When tipping, you consider the service and food quality,
rated between 0 and 10.  You use this to leave a tip of between 0 and 25%.

We would formulate this problem as:

* Antecednets (Inputs)
   - `service`
      * Universe (ie, crisp value range): How good was the service of the wait
        staff, on a scale of 0 to 10?
      * Fuzzy set (ie, fuzzy value range): poor, acceptable, amazing
   - `food quality`
      * Universe: How tasty was the food, on a scale of 0 to 10?
      * Fuzzy set: bad, decent, great
* Consequents (Outputs)
   - `tip`
      * Universe: How much should we tip, on a scale of 0% to 25%
      * Fuzzy set: low, medium, high
* Rules
   - IF the *service* was good  *or* the *food quality* was good,
     THEN the tip will be high.
   - IF the *service* was average, THEN the tip will be medium.
   - IF the *service* was poor *and* the *food quality* was poor
     THEN the tip will be low.
* Usage
   - If I tell this controller that I rated:
      * the service as 9.8, and
      * the quality as 6.5,
   - it would recommend I leave:
      * a 20.2% tip.


Creating the Tipping Controller Using the skfuzzy control API
-------------------------------------------------------------

We can use the `skfuzzy` control system API to model this.  First, let's
define fuzzy variables
"""
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import time



position_input = input("Enter how far from the final goal: ")
yawerror_input = input("Enter the error in yaw angle: ")


# New Antecedent/Consequent objects hold universe variables and membership
# functions
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
"""
.. image:: PLOT2RST.current_figure
"""
"""
.. image:: PLOT2RST.current_figure


Fuzzy rules
-----------

Now, to make these triangles useful, we define the *fuzzy relationship*
between input and output variables. For the purposes of our example, consider
three simple rules:

1. If the food is poor OR the service is poor, then the tip will be low
2. If the service is average, then the tip will be medium
3. If the food is good OR the service is good, then the tip will be high.

Most people would agree on these rules, but the rules are fuzzy. Mapping the
imprecise rules into a defined, actionable tip is a challenge. This is the
kind of task at which fuzzy logic excels.
"""

rule1 = ctrl.Rule(position['near'],                                                     velocity['low'])
rule2 = ctrl.Rule(position['medium_near'],                                              velocity['medium_low'])
rule3 = ctrl.Rule(position['medium'] | yawerror['turn_right'] | yawerror['turn_left'],  velocity['medium'])
rule4 = ctrl.Rule(position['medium_far'],                                               velocity['medium_high'])
rule5 = ctrl.Rule(position['far'],                                                      velocity['high'])
rule6 = ctrl.Rule(yawerror['slide_left'] | yawerror['slide_right'],                     yawrate['medium'])
rule7 = ctrl.Rule(yawerror['turn_left'] | yawerror['turn_right'],                       yawrate['high'])
rule8 = ctrl.Rule(yawerror['do_nothing'],                                               yawrate['low'])

rule1.view()

"""
.. image:: PLOT2RST.current_figure

Control System Creation and Simulation
---------------------------------------

Now that we have our rules defined, we can simply create a control system
via:
"""

velocity_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
yawrate_ctrl  = ctrl.ControlSystem([rule6, rule7, rule8])

"""
In order to simulate this control system, we will create a
``ControlSystemSimulation``.  Think of this object representing our controller
applied to a specific set of cirucmstances.  For tipping, this might be tipping
Sharon at the local brew-pub.  We would create another
``ControlSystemSimulation`` when we're trying to apply our ``tipping_ctrl``
for Travis at the cafe because the inputs would be different.
"""

controlling1 = ctrl.ControlSystemSimulation(velocity_ctrl)
controlling2 = ctrl.ControlSystemSimulation(yawrate_ctrl)

"""
We can now simulate our control system by simply specifying the inputs
and calling the ``compute`` method.  Suppose we rated the quality 6.5 out of 10
and the service 9.8 of 10.
"""
# Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
# Note: if you like passing many inputs all at once, use .inputs(dict_of_data)
controlling1.input['position'] = position_input
controlling1.input['yawerror'] = yawerror_input
controlling2.input['yawerror'] = yawerror_input

# Crunch the numbers
controlling1.compute()
controlling2.compute()

"""
Once computed, we can view the result as well as visualize it.
"""
print controlling1.output['velocity']
velocity.view(sim=controlling1)
print controlling2.output['yawrate']
yawrate.view(sim=controlling2)

"""
.. image:: PLOT2RST.current_figure

The resulting suggested tip is **20.24%**.

Final thoughts
--------------

The power of fuzzy systems is allowing complicated, intuitive behavior based
on a sparse system of rules with minimal overhead. Note our membership
function universes were coarse, only defined at the integers, but
``fuzz.interp_membership`` allowed the effective resolution to increase on
demand. This system can respond to arbitrarily small changes in inputs,
and the processing burden is minimal.

"""

def pause():
    programPause = raw_input("Press the <ENTER> key to continue...")

print("Check the output curves....")
pause()