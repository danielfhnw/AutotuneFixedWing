# -*- coding: utf-8 -*-
"""
Created on Tue Aug 16 14:36:07 2022

@author: ma1012017
"""

from pymavlink import mavutil
from sys import platform
import time


UDP = True

master = mavutil.mavlink_connection("udp:127.0.0.1:3000", baud=1000000)
    
master.wait_heartbeat()

master.set_mode(209, 458752)
print("stabilized mode")

t = time.time()
print("waiting")
while time.time() - t < 5:
    pass

master.set_mode_manual()
print("manual mode")


master.close()
