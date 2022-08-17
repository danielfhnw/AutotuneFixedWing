# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 15:45:54 2022

@author: ma1012017
"""

from pymavlink import mavutil
from sys import platform


if platform == "win32":
    master = mavutil.mavlink_connection("COM4")
else:
    master = mavutil.mavlink_connection("/dev/serial0", baud=57600)
    
msg = master.recv_match(type='BATTERY_STATUS', blocking=True)
print(msg)

master.close()