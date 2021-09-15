# -*- coding: utf-8 -*-
"""
Created on Wed Aug 11 21:35:52 2021

@author: Balazs
"""

import krpc
import time
import math

connection = krpc.connect()
###########################e

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
        diff_alt = (current+(360-target))*math.copysign(1, diff_normal)*(-1.0)
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
                               (abs(self.f_pitch_ctrl_by_pitch(roll))+abs(self.f_head_ctrl_by_pitch(roll))))
        
    def calc_distance_yaw(self, pitch_diff, heading_diff, roll):
        self.distance_yaw = ((pitch_diff*self.f_pitch_ctrl_by_yaw(roll)+
                                heading_diff*self.f_head_ctrl_by_yaw(roll))/
                               (abs(self.f_pitch_ctrl_by_pitch(roll))+abs(self.f_head_ctrl_by_pitch(roll))))
        
    def update_direction_vector(self, ref_frame, vessel=vessel):
        self.direction = [vessel.flight(ref_frame).pitch,
                          vessel.flight(ref_frame).heading,
                          vessel.flight(ref_frame).roll]        
        
    def f_control_input(self, difference, smoothness):
        # difference: difference in deggrees to the desired heading
        # smoothness: integer
        return math.copysign(1, difference*(-1))*(1-1/math.exp(abs(difference)/smoothness))
    #######
    def updateNumbers(self, target_vector, ref_frame, vessel=vessel):
       self.update_direction_vector(ref_frame, vessel)
       self.calc_pitch_diff(self.direction[0], target_vector[0])
       self.calc_heading_diff(self.direction[1], target_vector[1])
       self.calc_roll_diff(self.direction[2], target_vector[2])
       self.calc_distance_pitch(self.pitch_diff, self.heading_diff, self.direction[2])
       self.calc_distance_yaw(self.pitch_diff, self.heading_diff, self.direction[2])
           
    
    def controlAttitude(self, target_vector, ref_frame, vessel=vessel, smoothness=500, roll_rel_smoothness=0.1):
       self.updateNumbers(target_vector, ref_frame, vessel=vessel)
       
       if ((abs(self.distance_pitch)<5.0) & (abs(self.distance_yaw)<5.0) &
           (abs(self.roll_diff)<5.0)):
           vessel.control.pitch = 0
           vessel.control.yaw = 0
           vessel.control.roll = 0
           vessel.control.sas = True
       elif abs(self.roll_diff)>5.0:
           vessel.control.sas = False
           vessel.control.roll = self.f_control_input(self.roll_diff, smoothness*roll_rel_smoothness)
           vessel.control.pitch = 0
           vessel.control.yaw = 0
           vessel.control.sas = True
       else:
           vessel.control.sas = False
           vessel.control.roll = 0
           vessel.control.pitch = self.f_control_input(self.distance_pitch, smoothness)
           vessel.control.yaw = self.f_control_input(self.distance_yaw, smoothness)
                      
    


avionics = avionicsUnit()            


while True:    
    #avionics.updateNumbers([80.0,90.0], ref_surf)
    avionics.controlAttitude([50.0,90.0,0.0], ref_surf, smoothness=500)
    #print(avionics.distance_yaw)


avionics.updateNumbers([50.0,90.0,0.0], ref_surf)

avionics.direction[0]
avionics.direction[1]
avionics.direction[2]

avionics.pitch_diff
avionics.distance_pitch

avionics.heading_diff
avionics.distance_yaw

avionics.roll_diff

avionics.f_pitch_ctrl_by_pitch(avionics.direction[2])
avionics.f_head_ctrl_by_pitch(avionics.direction[2])

vessel.control.yaw=0.3

vessel.control.yaw=0
