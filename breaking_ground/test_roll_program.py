# -*- coding: utf-8 -*-
"""
Created on Sun Jul 11 16:39:55 2021

@author: Balazs
"""


import krpc
import time
import math

connection = krpc.connect()
###########################

def control_input(difference, smoothness):
    # difference: difference in deggrees to the desired heading
    # smoothness: integer
    return math.copysign(1, difference*(-1))*(1-1/math.exp(abs(difference)/smoothness))

def calc_roll_diff(current,target):
    diff_normal = current-target
    diff_inverted_abs = 360-abs(current)-abs(target)
    if abs(diff_normal)<=diff_inverted_abs:
        difference = diff_normal
    else:
        difference = diff_inverted_abs*math.copysign(1, target)
    return difference


vessel = connection.space_center.active_vessel

while True:
    # calculating inputs
    TARGET_ROLL=0.0
    current_roll = vessel.flight().roll
    #roll_diff = current_roll-TARGET_ROLL
    roll_diff= calc_roll_diff(current=current_roll, target=TARGET_ROLL)
    # maninpulating controls
    if abs(roll_diff)<5:
        vessel.control.roll=0.0
        #vessel.control.sas = True
    else:
        vessel.control.sas = False
        vessel.control.roll = control_input(difference=roll_diff, smoothness=100)
        