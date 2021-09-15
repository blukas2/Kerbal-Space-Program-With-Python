# -*- coding: utf-8 -*-
"""
Created on Sat Jul 10 09:41:44 2021

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
vessel.control.sas = True
vessel.control.activate_next_stage()
print('LIFTOFF!!!')
stage = 1

ascent_phase = True
floating_phase = False
going_up = True
descent_phase = False


ref_frame = vessel.orbit.body.reference_frame 

while ascent_phase | floating_phase | descent_phase:
    while ascent_phase:
        if vessel.thrust == 0.0:            
            vessel.control.activate_next_stage()            
            if stage==1:
                print('Stage 2 ignition!')
                stage = stage+1                
            else:
                ascent_phase = False
                floating_phase = True
                print('Floating phase')
    while floating_phase:
        altitude = vessel.flight().mean_altitude
        #apoapsis = vessel.orbit.apoapsis-3840168.571881427
        vertical_speed = vessel.flight(ref_frame).vertical_speed
        
        if going_up & (vertical_speed<0.0):
            going_up = False
            print('ApoK passed')
            
        if (going_up==False) & (altitude<90000):
            vessel.control.sas = False
            floating_phase=False
            descent_phase=True
            print('Reentry guidance enabled')            
    while descent_phase:
       altitude = vessel.flight().mean_altitude           
       if altitude>3000:
           # calculating inputs
           TARGET_PITCH=0.0
           TARGET_ROLL=0.0
           current_pitch = vessel.flight().pitch
           current_roll = vessel.flight().roll
           pitch_diff = current_pitch-TARGET_PITCH
           roll_diff = current_roll-TARGET_ROLL
           # maninpulating controls
           if (abs(pitch_diff)<5) & (abs(roll_diff)<5):
               vessel.control.sas = True
           else:
               vessel.control.sas = False
               if abs(roll_diff)>5:
                   vessel.control.roll = control_input(difference=roll_diff, smoothness=1000)
               elif abs(pitch_diff)>5:
                   vessel.control.pitch = control_input(difference=pitch_diff, smoothness=500)
               
       else:
           vessel.control.pitch=0.0
           vessel.control.roll=0.0
           vessel.control.sas = False
           time.sleep(0.1)
           vessel.control.activate_next_stage()
           descent_phase=False
                     
print('Computer guidance ended.')


