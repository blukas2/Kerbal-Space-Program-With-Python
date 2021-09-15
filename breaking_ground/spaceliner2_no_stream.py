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

vessel = connection.space_center.active_vessel
ref_frame = vessel.orbit.body.reference_frame
ref_surf = vessel.surface_reference_frame

# attitude control module

class avionicsUnit:
    def __init__(self):
        self.direction = [None, None, None]
        self.pitch_diff = 0.0
        self.heading_diff = 0.0
        self.roll_diff = 0.0
        self.distance_pitch = 0.0
        self.distance_yaw = 0.0
        
    def calc_pitch_diff(self,current,target):
        self.pitch_diff= current-target
        
    def calc_heading_diff(self,current,target):
        diff_normal = current-target
        diff_alt = current+(360-target)*math.copysign(1, diff_normal)*(-1.0)
        if abs(diff_normal)<abs(diff_alt):
            difference=diff_normal
        else:
            difference=diff_alt
        self.heading_diff = difference
        
    def calc_roll_diff(self, current,target):
        diff_normal = current-target
        diff_inverted_abs = 360-abs(current)-abs(target)
        if abs(diff_normal)<=diff_inverted_abs:
            difference = diff_normal
        else:
            difference = diff_inverted_abs*math.copysign(1, target)
        self.roll_diff = difference
        
    def f_pitch_ctrl_by_pitch(self, roll):
        return math.cos(roll*math.pi/180)
    
    def f_pitch_ctrl_by_yaw(self, roll):
        return -math.sin(roll*math.pi/180)
    
    def f_head_ctrl_by_yaw(self,roll):
        return math.cos(roll*math.pi/180)

    def f_head_ctrl_by_pitch(self, roll):
        return math.sin(roll*math.pi/180)
    
    def calc_distance_pitch(self, pitch_diff, heading_diff, roll):
        self.distance_pitch = ((pitch_diff*self.f_pitch_ctrl_by_pitch(roll)+
                                heading_diff*self.f_head_ctrl_by_pitch(roll))/
                               (self.f_pitch_ctrl_by_pitch(roll)+self.f_head_ctrl_by_pitch(roll)))
        
    def calc_distance_yaw(self, pitch_diff, heading_diff, roll):
        self.distance_yaw = ((pitch_diff*self.f_pitch_ctrl_by_yaw(roll)+
                                heading_diff*self.f_head_ctrl_by_yaw(roll))/
                               (self.f_pitch_ctrl_by_pitch(roll)+self.f_head_ctrl_by_pitch(roll)))
        
    def update_direction_vector(self, ref_frame, vessel=vessel):
        self.direction = [vessel.flight(ref_frame).pitch,
                          vessel.flight(ref_frame).heading,
                          vessel.flight(ref_frame).roll]        
        
    def f_control_input(self, difference, smoothness):
        # difference: difference in deggrees to the desired heading
        # smoothness: integer
        return math.copysign(1, difference*(-1))*(1-1/math.exp(abs(difference)/smoothness))
    #######
    def controlAttitude(self, target_vector, ref_frame, vessel=vessel, smoothness=500):
       self.update_direction_vector(ref_frame, vessel)
       self.calc_pitch_diff(self.direction[0], target_vector[0])
       self.calc_heading_diff(self.direction[1], target_vector[1])
       self.calc_distance_pitch(self.pitch_diff, self.heading_diff, self.direction[2])
       self.calc_distance_yaw(self.pitch_diff, self.heading_diff, self.direction[2])
       
       if (self.distance_pitch<5.0) & (self.distance_yaw<5.0):
           vessel.control.pitch = 0
           vessel.control.yaw = 0
           vessel.control.sas = True
       else:
           vessel.control.sas = False
           vessel.control.pitch = self.f_control_input(self.distance_pitch, smoothness)
           vessel.control.yaw = self.f_control_input(self.distance_yaw, smoothness)
           

avionics = avionicsUnit()            

           
##########################################

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
    



g = 9.8
TWR_DRY = 4.56
TWR_WET = 1.25
TWR_LANDED = 4.1
#TWR_LANDED = 3.5
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

while ascent_phase | floating_phase | descent_phase | landing_phase:
    # telemetry
    altitude = abs(vessel.flight().mean_altitude)-ALT_LANDED        
    vertical_speed = vessel.flight(ref_frame).vertical_speed
    fuel_curr = vessel.resources.amount('LiquidFuel')
    
    if ascent_phase:
        apoapsis = vessel.orbit.apoapsis-3840166.034551084
        # MECO
        if apoapsis>100000:
            vessel.control.throttle = 0
            
            ascent_phase=False
            floating_phase=True
            print('MECO')
    
    if floating_phase:        
        if going_up & (vertical_speed<0.0):
            going_up = False
            print('ApoK passed')
            
        if (going_up==False) & (altitude<90000):
            #vessel.control.sas = False
            floating_phase=False
            descent_phase=True
            print('Reentry guidance enabled')
    if descent_phase:        
        twr_act = calc_twr_act(TWR_DRY, TWR_WET, FUEL_MAX, fuel_curr)
        a = calc_a(twr_act, TWR_LANDED, g)
        target_altitude=calc_landing_burn_start(abs(vertical_speed), a)
        
        if altitude<target_altitude:
            vessel.control.throttle = 1
            descent_phase = False
            landing_phase = True
            print('LANDING BURN START!')
    if landing_phase:
        twr_act = calc_twr_act(TWR_DRY, TWR_WET, FUEL_MAX, fuel_curr)
        a = calc_a(twr_act, TWR_LANDED, g)
        
        if altitude>200:
            target_speed=calc_target_speed(altitude, a)*0.8
            throttle_control(abs(vertical_speed), target_speed, 0.3)
            #print(altitude)
        elif altitude>0.4:
            target_speed=5.0
            throttle_control(abs(vertical_speed), target_speed, 0.1)
        else:
            vessel.control.throttle = 0
            landing_phase = False
            print('touchdown altitude is: ' + str(altitude))
            
        # deploy landing legs
        if gearsUp & (altitude<200):
            vessel.control.gear=False
            vessel.control.gear=True
            gearsUp=False


print('Touchdown!')
print('Computer guidance ended.')
            

