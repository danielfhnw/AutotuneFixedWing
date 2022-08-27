# -*- coding: utf-8 -*-
"""
Created on Tue Aug 16 14:36:07 2022

@author: ma1012017
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection("udp:127.0.0.1:3000", baud=500000)
    
master.wait_heartbeat()

t = time.time()
print("starting override")
master.mav.rc_channels_override_send(master.target_system, master.target_component, 
                                   1100, 0, 0, 0, 0, 0, 0, 0)
while time.time() - t < 10:
    pass

#master.mav.command_long_send(master.target_system, master.target_component,
#                                   mavutil.mavlink.RC_CHANNELS_OVERRIDE, 
#                                   0, 0, 0, 0, 0, 0, 0, 0)
master.mav.rc_channels_override_send(master.target_system, master.target_component, 
                                   0, 0, 0, 0, 0, 0, 0, 0)

print("stopped override")    
master.close()
