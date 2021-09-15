# -*- coding: utf-8 -*-
"""
Created on Fri Jul 23 21:19:35 2021

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


def calc_landing_burn_start(v, a):
    alt = (v**2.0)/(2.0*a)
    return alt

def calc_target_speed(alt, a):
    v = (2.0*a*alt)**(1.0/2.0)
    return v

def calc_a(twr_act, twr_landed, g):
    a = ((twr_act+twr_landed)/2)*g-g
    return a

def calc_twr_act(twr_dry, twr_wet, fuel_max, fuel_curr):
    twr_act = (twr_dry-twr_wet)*((fuel_max-fuel_curr)/fuel_max)+twr_wet
    return twr_act


def throttle_control(current_speed, target_speed, min_throttle):
    if current_speed>target_speed:
        vessel.control.throttle = min(vessel.control.throttle+0.05,1)
    if current_speed<target_speed:
        vessel.control.throttle = max(vessel.control.throttle-0.05, min_throttle)
    

vessel = connection.space_center.active_vessel
ref_frame = vessel.orbit.body.reference_frame

g = 9.8
TWR_DRY = 4.56
TWR_WET = 1.48
#TWR_LANDED = 4.1
TWR_LANDED = 3.5
ALT_LANDED = 166.0322
FUEL_MAX = vessel.resources.max('LiquidFuel')

###########################
vessel.control.sas = True
vessel.control.throttle = 1
vessel.control.activate_next_stage()



print('LIFTOFF!!!')

ascent_phase = True
floating_phase = False
going_up = True
descent_phase = False
landing_phase = False

gearsUp = True


altitude = connection.add_stream(getattr, vessel.flight(), 'mean_altitude')
vertical_speed = connection.add_stream(getattr, vessel.flight(ref_frame), 'vertical_speed')
fuel_curr = connection.add_stream(vessel.resources.amount, 'LiquidFuel')

while ascent_phase | floating_phase | descent_phase | landing_phase:
    # telemetry
    #altitude = vessel.flight().mean_altitude-ALT_LANDED        
    #vertical_speed = vessel.flight(ref_frame).vertical_speed
    #fuel_curr = vessel.resources.amount('LiquidFuel')
    
    if ascent_phase:
        apoapsis = vessel.orbit.apoapsis-3840166.034551084
        # MECO
        if apoapsis>100000:
            vessel.control.throttle = 0
            
            ascent_phase=False
            floating_phase=True
            print('MECO')
    
    if floating_phase:
        print(vertical_speed())
        time.sleep(1)
        
        if going_up & (vertical_speed()<0.0):
            going_up = False
            print('ApoK passed')
            
        if (going_up==False) & (altitude()<(90000+ALT_LANDED)):
            #vessel.control.sas = False
            floating_phase=False
            descent_phase=True
            print('Reentry guidance enabled')
    if descent_phase:        
        twr_act = calc_twr_act(TWR_DRY, TWR_WET, FUEL_MAX, fuel_curr())
        a = calc_a(twr_act, TWR_LANDED, g)
        target_altitude=calc_landing_burn_start(abs(vertical_speed()), a)
        
        if altitude()<(target_altitude+ALT_LANDED):
            vessel.control.throttle = 1
            descent_phase = False
            landing_phase = True
            print('LANDING BURN START!')
    if landing_phase:
        twr_act = calc_twr_act(TWR_DRY, TWR_WET, FUEL_MAX, fuel_curr())
        a = calc_a(twr_act, TWR_LANDED, g)
        
        if altitude()>(300.0+ALT_LANDED):
            target_speed=calc_target_speed(altitude()-ALT_LANDED, a)
            throttle_control(abs(vertical_speed()), target_speed, 0.2)
        else:
            print('brace for impact') # for testing
            
            target_speed=15.0
            throttle_control(abs(vertical_speed()), target_speed, 0.2)
        if gearsUp & (altitude()<(200.0+ALT_LANDED)):
            vessel.control.gear=False
            vessel.control.gear=True
            gearsUp=False
        if altitude()<(1.0+ALT_LANDED):
            vessel.control.throttle = 0
            landing_phase = False

print('Touchdown!')
print('Computer guidance ended.')
            

