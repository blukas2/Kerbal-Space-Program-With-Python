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
            print('Stage 2 ignition!')
            vessel.control.activate_next_stage()            
            if stage==1:
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
            vessel.auto_pilot.engage()
            floating_phase=False
            descent_phase=True
            
            print('Reentry guidance enabled')
            
    while descent_phase:
       altitude = vessel.flight().mean_altitude           
       if altitude>3000:
           vessel.auto_pilot.target_pitch=0
           vessel.auto_pilot.target_roll=0
       else:
           vessel.auto_pilot.disengage()
           time.sleep(0.1)
           vessel.control.activate_next_stage()
           descent_phase=False
                     
print('Computer guidance ended.')


