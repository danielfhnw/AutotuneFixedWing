# -*- coding: utf-8 -*-
"""
Created on Tue Aug 16 14:36:07 2022

@author: ma1012017
"""

from pymavlink import mavutil
from sys import platform

UDP = True


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
elif UDP:
    master = mavutil.mavlink_connection("udp:127.0.0.1:3000", baud=500000)
else:
    master = mavutil.mavlink_connection("/dev/serial0", baud=57600)
    
master.wait_heartbeat()
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_TARGET, 1000)
print("connected")

while True:
    try:
        msg = master.recv_match(type='ATTITUDE_TARGET', blocking=True)
        print(msg.body_roll_rate)
    except KeyboardInterrupt:
        print("Program stopped")
        break
 
master.close()