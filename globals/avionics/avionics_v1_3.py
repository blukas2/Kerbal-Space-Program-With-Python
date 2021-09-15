# -*- coding: utf-8 -*-
"""
Created on Sun Aug 29 14:33:20 2021

@author: Balazs
"""


# requires vessel
# requires time

class angularVelocityMonitor():
    def __init__(self):
        self.__prev_time = 0.0        
        self.__prev_roll = 0.0        
        self.av_roll = 0.0
        
    def calc_roll_change(self, current, prev):
        # same as calc roll diff
        diff_normal = current-prev
        diff_inverted_abs = 360-abs(current)-abs(prev)
        if abs(diff_normal)<=diff_inverted_abs:
            difference = diff_normal
        else:
            difference = diff_inverted_abs*math.copysign(1, prev)
        return difference
        
    def get_angvel(self, direction, resolution=5.0):
        # calculate angular velocity in degrees per second
        curr_time = time.time()        
        curr_roll = direction[2]
        
        if (curr_time-self.__prev_time)>(1.0/resolution):
            if ((self.__prev_time==0.0) | 
                ((curr_time-self.__prev_time)>1.0/resolution*2.0)):
                self.av_roll=0.0
            else:                
                #self.av_roll = (curr_roll-self.__prev_roll)*resolution
                self.av_roll = self.calc_roll_change(curr_roll, self.__prev_roll)*resolution
                #print(self.av_roll)
            self.__prev_time = curr_time
            self.__prev_roll = curr_roll
            


class avionicsUnit:
    def __init__(self):
        self.direction = [None, None, None]
        self.pitch_diff = 0.0
        self.heading_diff = 0.0
        self.roll_diff = 0.0
        self.distance_pitch = 0.0
        self.distance_yaw = 0.0
        
        self.av_unit = angularVelocityMonitor()
        
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
           
    
    def controlAttitude(self, target_vector, ref_frame, vessel=vessel, 
                        smoothness=500, roll_rel_smoothness=0.1, 
                        precision=5.0, roll_precision=5.0):
       self.updateNumbers(target_vector, ref_frame, vessel=vessel)
       
       if ((abs(self.distance_pitch)<precision) & (abs(self.distance_yaw)<precision) &
           (abs(self.roll_diff)<roll_precision)):
           vessel.control.pitch = 0
           vessel.control.yaw = 0
           vessel.control.roll = 0
           vessel.control.sas = True
       else:
           vessel.control.sas = False
           vessel.control.pitch = self.f_control_input(self.distance_pitch, smoothness)
           vessel.control.yaw = self.f_control_input(self.distance_yaw, smoothness)

           self.av_unit.get_angvel(self.direction)
           if (((abs(self.roll_diff)<roll_precision) & (abs(self.av_unit.av_roll)>0.1)) |
               ((abs(self.roll_diff)<10.0) & (abs(self.av_unit.av_roll)>2.0))):
               vessel.control.roll = self.f_control_input(self.av_unit.av_roll, smoothness*roll_rel_smoothness/10.0)
               print('angvel: ' + str(self.av_unit.av_roll) + '; roll input: ' + str(vessel.control.roll))
           else:
               vessel.control.roll = self.f_control_input(self.roll_diff, smoothness*roll_rel_smoothness)                
           vessel.control.sas = True

avionics = avionicsUnit()            
