# -*- coding: utf-8 -*-
"""
Created on Tue Aug 10 23:08:36 2021

@author: Balazs
"""

import krpc
import math
import time

def cross(u,v):
    return (
        u[1]*v[2] - u[2]*v[1],
        u[2]*v[0] - u[0]*v[2],
        u[0]*v[1] - u[1]*v[0])

def dot(u,v):
    return u[0]*v[0]+u[1]*v[1]+u[2]*v[2]

conn = krpc.connect()

vessel = conn.space_center.active_vessel

while True:

    # these vectors are all in vessel.surface_reference_frame
    north = (0,1,0)
    east = (0,0,1)
    prograde = vessel.flight().prograde

    # normal vector defining the plane (note that ksp uses a left handed coordinate system)    
    plane_normal = cross(north,east)

    # angle between plane and prograde vector (in degrees)
    print(str(math.asin(dot(plane_normal, prograde)) * (180.0/math.pi)))

    #conn.space_center.clear_drawing()
    #conn.space_center.draw_direction(north,vessel.surface_reference_frame,(1,0,0))
    #conn.space_center.draw_direction(east,vessel.surface_reference_frame,(0,1,0))
    #conn.space_center.draw_direction(plane_normal,vessel.surface_reference_frame,(1,1,1))
    #conn.space_center.draw_direction(prograde,vessel.surface_reference_frame,(0,0,1))
    time.sleep(1)