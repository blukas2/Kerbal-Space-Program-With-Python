# -*- coding: utf-8 -*-
"""
Created on Sat Aug 28 18:04:18 2021

@author: Balazs
"""


import krpc
import time
import math


#############
connection = krpc.connect()


vessel = connection.space_center.active_vessel

space_center = connection.space_center

ref_orbit = vessel.orbit.body.reference_frame
ref_body = vessel.orbit.body.non_rotating_reference_frame
ref_surf = vessel.surface_reference_frame

print(vessel.angular_velocity(ref_orbit))
print(vessel.angular_velocity(ref_body))
print(vessel.angular_velocity(ref_surf))


ref_frame = ref_body
angvel = vessel.angular_velocity(ref_frame)
print(angvel)
print(space_center.transform_direction(angvel, ref_frame, vessel.reference_frame))


roll_prev = vessel.flight(ref_surf).roll
while True:
    roll_curr = vessel.flight(ref_surf).roll
    rollspeed = roll_curr - roll_prev
    print(rollspeed)
    roll_prev = roll_curr
#    time.sleep(1)


old = time.time()
new = time.time()
elapsed = new - old
print(elapsed)
