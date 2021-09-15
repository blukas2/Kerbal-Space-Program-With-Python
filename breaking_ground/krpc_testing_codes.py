# -*- coding: utf-8 -*-
"""
Created on Thu Jul 29 21:45:03 2021

@author: Balazs
"""
import krpc
import time
import math

connection = krpc.connect()
vessel = connection.space_center.active_vessel
ref_frame = vessel.orbit.body.reference_frame
ref_frame2 = vessel.surface_reference_frame

vessel.flight(ref_frame).retrograde

vessel.flight(ref_frame).prograde

vessel.flight().direction[0]

vessel.flight(ref_frame).pitch

vessel.flight().pitch

connection.space_center.draw_direction(velocity, ref_frame, (1,0,0))

print(vessel.flight(ref_frame).direction[0]/math.pi*180)
print(vessel.flight(ref_frame).direction[1]/math.pi*180)
print(vessel.flight(ref_frame).direction[2]/math.pi*180)


print(vessel.flight(ref_frame2).direction[0]/math.pi*180)
print(vessel.flight(ref_frame2).direction[1]/math.pi*180)
print(vessel.flight(ref_frame2).direction[2]/math.pi*180)


vessel.flight(ref_frame2).direction

vessel.flight().direction

#######################################
connection.space_center.Part.landing_leg.Part

connection.space_center.LandingLeg.set_deployed(1)

vessel.parts.with_title('LT-05 Micro Landing Strut').set_deployed(1)

for leg in vessel.parts.landing_legs:
    leg.deployed=True

#.Deployed()

vessel.parts.landing_gear

vessel.parts.landing_legs

vessel.parts.landing_legs[0].deployed=0
vessel.parts.landing_legs[0].state            
            
vessel.control.gear=False
vessel.control.gear=True

#################################
vessel.control.sas = True

vessel.control.sas = False

vessel.control.sas_mode = vessel.auto_pilot.sas_mode.retrograde

vessel.auto_pilot.engage()
vessel.auto_pilot.disengage()

vessel.auto_pilot.target_pitch_and_heading(90.0,0.0)

###################################

