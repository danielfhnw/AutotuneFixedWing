# -*- coding: utf-8 -*-
"""
Created on Tue Aug 16 14:36:07 2022

@author: ma1012017
"""

from pymavlink import mavutil

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
    

master = mavutil.mavlink_connection("udp:127.0.0.1:3000", baud=1000000)
    
master.wait_heartbeat()
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 10)

reset = False

while not reset:
    msg = master.recv_match(type=['HEARTBEAT'])
    if msg is not None:
        if msg.get_type() == 'HEARTBEAT':
            if msg.custom_mode != 67371008 and msg.autopilot == 12:
                reset = True
                master.param_set_send("FW_RR_FF", 0.5)
                master.param_set_send("FW_RR_P", 0.05)
                master.param_set_send("FW_RR_I", 0.1)
                print("reset parameters")

master.close()
