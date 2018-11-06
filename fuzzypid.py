#! /usr/bin/python

# -*- coding: utf-8 -*-
"""
@author: acs
"""

import skfuzzy as fuzz
from skfuzzy import control as ctrl
import acspid
import control
import numpy as np
from matplotlib import pyplot as plt

plt.ion()
fig=plt.figure()

ferr = ctrl.Antecedent(np.arange(-150, 150, 1), 'ferr')
fder = ctrl.Antecedent(np.arange(-150, 150, 1), 'fder')
fout = ctrl.Consequent(np.arange(-1, 1, 0.01), 'fout')

ferr.automf(5)
fder.automf(5)
fout.automf(5)
fout['poor'] = fuzz.trimf(fout.universe, [-1, -1, -0.5])
fout['mediocre'] = fuzz.trimf(fout.universe, [-1, -0.5, 0])
fout['average'] = fuzz.trimf(fout.universe, [-0.1, 0, 0.1])
fout['decent'] = fuzz.trimf(fout.universe, [0, 0.5, 2])
fout['good'] = fuzz.trimf(fout.universe, [0.5, 1, 1])
fout.view()
ferr.view()
fder.view()
plt.show()
plt.pause(0.0001)

#'poor'; 'mediocre'; 'average'; 'decent', or 'good'
rules=[]
rules.append(ctrl.Rule(ferr['average'] | fder['average'] , fout['average']))
rules.append(ctrl.Rule(ferr['decent'] | fder['decent'] , fout['decent']))
rules.append(ctrl.Rule(ferr['good'] | fder['good'] , fout['good']))
rules.append(ctrl.Rule(ferr['mediocre'] | fder['mediocre'] , fout['mediocre']))
rules.append(ctrl.Rule(ferr['poor'] | fder['poor'] , fout['poor']))

fctrl = ctrl.ControlSystem(rules)
fpid = ctrl.ControlSystemSimulation(fctrl)

pid=acspid.pidcont(1.2,0.02,0.01,5,-5)

pid2=acspid.pidcont(1.2,0.02,0.01,5,-5)

d=np.zeros(10)
for i in range(10):
    d=np.append(d,np.ones(10)*np.random.uniform(-100,100,1))

print len(d)
m=[]
m.append(0.0)
m2=[]
m2.append(0.0)
e=[]
de=[]
e2=[]
de2=[]

kp=pid.kp
kd=pid.kd
ki=pid.ki
for i in range(len(d)):
    pid.setDesired(d[i])
    print "e:",pid.error ,"\t de:", pid.ed
    fpid.input['ferr'] = pid.error
    fpid.input['fder'] = pid.ed
    fpid.compute()
    newpid=np.abs(fpid.output['fout'])
    print "PID:", newpid*pid.kp,"\t",newpid*pid.ki,"\t",newpid*pid.kd
    pid.setGains(newpid*kp,newpid*ki,newpid*kd)
    newm=pid.update(m[-1])
    newm=m[-1]+newm
    print i,m[-1],newm
    m.append(newm)
    e.append(pid.error)
    de.append(pid.ed)

    pid2.setDesired(d[i])
    newm2=pid2.update(m2[-1])
    newm2=m2[-1]+newm2
    m2.append(newm2)
    e2.append(pid2.error)
    de2.append(pid2.ed)

    ax1 =plt.subplot(2,1,1)
    ax1.set_xlim([0, len(d)])
    ax1.set_ylim([-200, 200])
    plt.grid()
    plt.plot(range(len(m)),m,linewidth=5.0)
    plt.plot(range(len(m2)),m2,linewidth=2.0)
    plt.plot(range(len(d)),d,'g--')

    plt.title('Status')
    ax2=plt.subplot(2,1,2)
    ax2.set_xlim([0, 50])
    ax2.set_ylim([-100, 100])
    plt.plot(range(len(e)),e,'r-',range(len(de)),de,'g-')
    plt.grid()
    plt.title('e and ed')
    #plt.draw()
    plt.show()
    plt.pause(0.0001)