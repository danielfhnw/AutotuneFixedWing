# -*- coding: utf-8 -*-
"""
Created on Tue Aug 16 14:36:07 2022

@author: ma1012017
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection("udp:127.0.0.1:3000", baud=500000)
    
master.wait_heartbeat()

master.set_mode(81, 458752)
print("stabilized mode")

t = time.time()
print("waiting")
while time.time() - t < 10:
    msg = master.recv_match(type="HEARTBEAT")
    if msg is not None:
        print(msg)
    master.set_mode(81, 458752)

master.set_mode_manual()
print("manual mode")


master.close()
