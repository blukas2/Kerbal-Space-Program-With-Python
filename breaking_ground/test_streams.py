# -*- coding: utf-8 -*-
"""
Created on Sun Aug  1 09:04:44 2021

@author: Balazs
"""

import krpc
import time
import math

connection = krpc.connect()

vessel = connection.space_center.active_vessel
ref_frame = vessel.orbit.body.reference_frame


vertical_speed = connection.add_stream(getattr, vessel.flight(ref_frame), 'vertical_speed')
altitude = connection.add_stream(getattr, vessel.flight(), 'mean_altitude')

#altitude = connection.stream(getattr(vessel.flight(), 'mean_altitude'))

vessel.control.sas = True
vessel.control.throttle = 1
vessel.control.activate_next_stage()

flight=True


# telemetry


# =============================================================================
# while flight:
#     if altitude()>2000.0:
#         vessel.control.throttle = 0
#         flight=False
#         
# print('Program ended.')
# 
# =============================================================================

while flight:
    while altitude()<500:
        print(altitude())
    vessel.control.throttle = 0
    flight=False
        
print('Program ended.')
