# -*- coding: utf-8 -*-
"""
Created on Thu Aug  5 22:10:40 2021

@author: Balazs
"""

vessel_direction = (0.7158184726046639, 0.01497484676382465, 0.6981258255087875)

def get_pitch_heading_roll(input_vector):
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
    # Compute the roll
    # Compute the plane running through the vessels direction and the upwards direction
    up = (1,0,0)
    plane_normal = cross_product(vessel_direction, up)
    # Compute the upwards direction of the vessel
    vessel_up = conn.space_center.transform_direction((0,0,-1), vessel.reference_frame, vessel.surface_reference_frame)
    # Compute the angle between the upwards direction of the vessel and the plane
    roll = angle_between_vector_and_plane(vessel_up, plane_normal)
    # Adjust so that the angle is between -180 and 180 and
    # rolling right is +ve and left is -ve
    if vessel_up[0] > 0:
        roll *= -1
    elif roll < 0:
        roll += 180
    else:
        roll -= 180
        
    return [pitch,heading,roll]

get_pitch_heading_roll(vessel_direction)



vessel_direction = (0.7158184726046639, 0.01497484676382465, 0.6981258255087875)

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

get_pitch_heading(vessel_direction)



import krpc
import time
import math

connection = krpc.connect()
###########################

vessel = connection.space_center.active_vessel
ref_orb = vessel.orbit.body.reference_frame
ref_surf = vessel.surface_reference_frame

vessel.flight(ref_surf).retrograde
vessel.flight(ref_surf).prograde

vessel.flight(ref_surf).direction

vessel.flight(ref_orb).prograde

get_pitch_heading(vessel.flight(ref_surf).retrograde)

get_pitch_heading(vessel.flight(ref_surf).retrograde)

get_pitch_heading(vessel.flight(ref_surf).prograde)

get_pitch_heading(vessel.flight(ref_orb).prograde)

get_pitch_heading(vessel.flight(ref_surf).direction)

vessel.flight(ref_surf).pitch
vessel.flight(ref_surf).heading
vessel.flight(ref_surf).roll


while True:
   get_pitch_heading(vessel.flight(ref_surf).retrograde)
   time.sleep(1)
   
# around 90 degrees the pitch difference is around -40
# around 120 degrees the heading difference is around +150
