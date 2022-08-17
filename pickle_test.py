# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 16:09:48 2022

@author: ma1012017
"""

from pymavlink import mavutil
from sys import platform
import time
import pickle


def request_message_interval(message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """

    if frequency_hz > 0:
        freq_out = 1e6 / frequency_hz
    elif frequency_hz == 0:
        freq_out = 0
    else:
        freq_out = -1

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id,  # The MAVLink message ID
        freq_out,  # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0,  # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        0, 0, 0, 0)
    

if platform == "win32":
    master = mavutil.mavlink_connection("COM4")
else:
    master = mavutil.mavlink_connection("/dev/serial0", baud=57600)
    
master.wait_heartbeat()
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 1000)
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 1000)


i = 0
t = time.time()

log = []

while i < 1000:
    msg = master.recv_match(type=['ATTITUDE', 'SERVO_OUTPUT_RAW'], blocking=True)
    log.append(msg)
    i = i+1

freq = 1000 / (time.time()-t)
print(f"Frequency: {freq} Hz")

master.close()

with open("log.pickle", 'wb') as f:
    # indent=2 is not needed but makes the file human-readable 
    # if the data is nested
    pickle.dump(log, f)
