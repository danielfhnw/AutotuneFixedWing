# -*- coding: utf-8 -*-
"""
Created on Tue Aug 16 14:36:07 2022

@author: ma1012017
"""

from pymavlink import mavutil
import time
import math


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
print("starting override")

while time.time() - t < 10:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        187,
        0,
        1, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan)
    master.set_mode_px4("OFFBOARD", None, None)
    time.sleep(0.1)

print("stopped override")    
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
