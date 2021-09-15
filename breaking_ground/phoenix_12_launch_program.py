# -*- coding: utf-8 -*-
"""
Created on Tue Sep 14 20:24:58 2021

@author: Balazs
"""


import krpc
import time
import math


#############
connection = krpc.connect()

vessel = connection.space_center.active_vessel
ref_orbit = vessel.orbit.body.reference_frame
ref_surf = vessel.surface_reference_frame

######################
# LAUNCH PROFILE VARIABLES

TARGET_APOAPSIS = 250000.0

HEADING = 90.0
POLAR_ORBIT = False

#HEADING = 356.0
#POLAR_ORBIT = True



ROLL_ORIENTATION = 180.0

PITCH_PROGRAM_START = 1000.0
PITCH_PROGRAM_45 = 25000.0
PITCH_PROGRAM_COMPLETE = 140000.0



#############################

def calc_target_pitch(altitude, 
                      start_altitude=PITCH_PROGRAM_START, 
                      altitude45=PITCH_PROGRAM_45, 
                      complete_altitude=PITCH_PROGRAM_COMPLETE):
    if altitude>complete_altitude:
        target_pitch=0.0
    elif altitude>altitude45:
        target_pitch = (complete_altitude-altitude)/(complete_altitude-altitude45)*45.0
    elif altitude>start_altitude:
        target_pitch = (altitude45-altitude)/(altitude45-start_altitude)*45.0+45.0
    return min(85.0,target_pitch)


def zeroout_controls(vessel=vessel):
    vessel.control.pitch = 0
    vessel.control.yaw = 0
    vessel.control.roll = 0
    vessel.control.sas = True


################################




# startup avionics computer

exec(open('E:/Software Engineering/Kerbal-Space-Program-With-Python/globals/avionics/avionics_v1_33.py').read())


ASCENT = True
COASTING = False
ORBITAL_INSERTION = False

PITCH_PROGRAM = False

FIRST_STAGE = True
SECOND_STAGE = False
THIRD_STAGE = False


vessel.control.sas = True
vessel.control.throttle = 1
vessel.control.activate_next_stage()

print('LIFTOFF!!!')


while ASCENT | COASTING | ORBITAL_INSERTION:
    altitude = abs(vessel.flight(ref_surf).mean_altitude)
    apoapsis = vessel.orbit.apoapsis-3840166.034551084
    
    if ASCENT:
        # orienting towards the desired orbit
        if (altitude>PITCH_PROGRAM_START) & (PITCH_PROGRAM==False):
            if vessel.flight(ref_surf).pitch>85.0:
                vessel.control.sas = False
                if POLAR_ORBIT:
                    vessel.control.yaw = 0.1
                else:
                    vessel.control.pitch = 0.1
            else:
                vessel.control.pitch = 0.0
                vessel.control.sas = True
                print('Pitch program start')
                PITCH_PROGRAM = True
        if PITCH_PROGRAM:
            avionics.controlAttitude(target_vector = [calc_target_pitch(altitude), HEADING, ROLL_ORIENTATION],
                                     ref_frame = ref_surf, 
                                     smoothness=10, roll_rel_smoothness=1.0, 
                                     precision=2.0, roll_precision=3.0)
            
        # staging
        if FIRST_STAGE:
            if vessel.thrust == 0.0:
                zeroout_controls()
                vessel.control.throttle=0.05
                time.sleep(0.5)
                print('Stage separation!')
                vessel.control.activate_next_stage()
                time.sleep(3.0)
                vessel.control.throttle=1
                FIRST_STAGE=False
                SECOND_STAGE=True
        if SECOND_STAGE:
            if vessel.thrust == 0.0:
                zeroout_controls()
                time.sleep(0.5)
                print('Stage separation!')
                vessel.control.activate_next_stage()
                time.sleep(1.0)
                SECOND_STAGE=False
                THIRD_STAGE=True
        # Engine cutoff
        if THIRD_STAGE:            
            if apoapsis>TARGET_APOAPSIS*0.99:
                vessel.control.throttle=0.3
            if apoapsis>TARGET_APOAPSIS:
                vessel.control.throttle=0
                print('Engine Shutdown')
                zeroout_controls()
                ASCENT=False
                COASTING = True
        
    if COASTING:
        if vessel.orbit.time_to_apoapsis<90.0:
            avionics.controlAttitude(target_vector = [0.0, HEADING, ROLL_ORIENTATION],
                                     ref_frame = ref_surf, 
                                     smoothness=100, roll_rel_smoothness=0.3, precision=2.0)
        if vessel.orbit.time_to_apoapsis<20.0:
            COASTING=False
            ORBITAL_INSERTION = True
            print('Engine Ignition!')
    if ORBITAL_INSERTION:
        avionics.controlAttitude(target_vector = [0.0, HEADING, ROLL_ORIENTATION],
                                     ref_frame = ref_surf, 
                                     smoothness=100, roll_rel_smoothness=0.3, precision=2.0)        
        periapsis = vessel.orbit.periapsis-3840166.034551084
                
        if periapsis<0.9*TARGET_APOAPSIS:
            vessel.control.throttle=1
        elif periapsis<TARGET_APOAPSIS:
            vessel.control.throttle=0.2
        else:
            vessel.control.throttle=0
            zeroout_controls()
            ORBITAL_INSERTION=False
            print('Orbital insertion complete!')
print('Computer guidance ended.')
            