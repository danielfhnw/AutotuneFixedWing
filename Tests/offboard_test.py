# -*- coding: utf-8 -*-
"""
Created on Tue Aug 16 14:36:07 2022

@author: ma1012017
"""

from pymavlink import mavutil
import time


master = mavutil.mavlink_connection("udp:127.0.0.1:3000", baud=500000)
    
master.wait_heartbeat()


master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

t = time.time()
print("starting target send")

targetreached = True
starttime = time.time()
lasttime = time.time()
rc_in = False


while targetreached and time.time()-starttime < 10 and not rc_in:
    if time.time() - lasttime > 0.1:
        lasttime = time.time()
        master.mav.set_attitude_target_send(
            int(time.time()), master.target_system,
            master.target_component,
            0b010000000, # ignore throttle
            [0,0,0,0], # no quaternions
            0.5, 0, 0, # body roll 
            0.2, # thrust
            [0,0,0]) # no 3D thrust
    #master.set_mode_px4("OFFBOARD", None, None)
    msg = master.recv_match(type=["ATTITUDE", "RC_CHANNELS"])
    if msg is not None:
        if msg.get_type() == "ATTITUDE":
            if msg.roll > 0.2:
                targetreached = False
        elif msg.get_type() == "RC_CHANNELS":
            if msg.chan3_raw-1500 > 100 or msg.chan3_raw-1500 < (-100):
                rc_in = True
                print("stopped due to manual input")
                master.mav.set_attitude_target_send(
                    int(time.time()), master.target_system,
                    master.target_component,
                    0b010000000, # ignore throttle
                    [0,0,0,0], # no quaternions
                    0, 0, 0, # body roll 
                    0.2, # thrust
                    [0,0,0]) # no 3D thrust
            
    master.set_mode_px4("OFFBOARD", None, None)


print("stopped target send")   
master.set_mode_px4("MANUAL", None, None)
print("set manual mode") 
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)
print("Waiting for the vehicle to disarm")
master.motors_disarmed_wait()
print('Disarmed!')

master.close()
