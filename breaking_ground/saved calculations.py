# -*- coding: utf-8 -*-
"""
Created on Wed Aug 11 21:26:22 2021

@author: Balazs
"""


def calc_roll_diff(current,target):
    diff_normal = current-target
    diff_inverted_abs = 360-abs(current)-abs(target)
    if abs(diff_normal)<=diff_inverted_abs:
        difference = diff_normal
    else:
        difference = diff_inverted_abs*math.copysign(1, target)
    return difference

def calc_pitch_diff(current,target):
    return current-target

def calc_heading_diff(current,target):
    diff_normal = current-target
    diff_alt = current+(360-target)*math.copysign(1, diff_normal)*(-1.0)
    if abs(diff_normal)<abs(diff_alt):
        difference=diff_normal
    else:
        difference=diff_alt
    return difference

def calc_pitch_ctrl_by_pitch(roll):
    return math.cos(roll*math.pi/180)

def calc_pitch_ctrl_by_yaw(roll):
    return -math.sin(roll*math.pi/180)

def calc_head_ctrl_by_yaw(roll):
    return math.cos(roll*math.pi/180)

def calc_head_ctrl_by_pitch(roll):
    return math.sin(roll*math.pi/180)

def distance_pitch(pitch_diff, heading_diff, roll):
    return ((pitch_diff*calc_pitch_ctrl_by_pitch(roll)+
            heading_diff*calc_head_ctrl_by_pitch(roll))/
            (calc_pitch_ctrl_by_pitch(roll)+calc_head_ctrl_by_pitch(roll)))
    
    

def control_input(difference, smoothness):
    # difference: difference in deggrees to the desired heading
    # smoothness: integer
    return math.copysign(1, difference*(-1))*(1-1/math.exp(abs(difference)/smoothness))



##########################################
def get_pitch_heading(input_vector):
    def cross_product(x, y):
        return (x[1]*y[2] - x[2]*y[1], x[2]*y[0] - x[0]*y[2], x[0]*y[1] - x[1]*y[0])
    def dot_product(x, y):
        return x[0]*y[0] + x[1]*y[1] + x[2]*y[2]    
    def magnitude(x):
        return math.sqrt(x[0]**2 + x[1]**2 + x[2]**2)
    def angle_between_vectors(x, y):
        """ Compute the angle between vector x and y """
        dp = dot_product(x, y)
        if dp == 0:
            return 0
        xm = magnitude(x)
        ym = magnitude(y)
        return math.acos(dp / (xm*ym)) * (180. / math.pi)
    def angle_between_vector_and_plane(x, n):
        """ Compute the angle between a vector x and plane with normal vector n """
        dp = dot_product(x,n)
        if dp == 0:
            return 0
        xm = magnitude(x)
        nm = magnitude(n)
        return math.asin(dp / (xm*nm)) * (180. / math.pi)

    vessel_direction = input_vector
    # Get the direction of the vessel in the horizon plane
    horizon_direction = (0, vessel_direction[1], vessel_direction[2])

    # Compute the pitch - the angle between the vessels direction and the direction in the horizon plane
    pitch = angle_between_vectors(vessel_direction, horizon_direction)
    if vessel_direction[0] < 0:
        pitch = -pitch
        
    # Compute the heading - the angle between north and the direction in the horizon plane
    north = (0,1,0)
    heading = angle_between_vectors(north, horizon_direction)
    if horizon_direction[2] < 0:
        heading = 360 - heading        
    return [pitch,heading]

