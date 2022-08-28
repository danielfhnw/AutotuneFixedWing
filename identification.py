# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 21:03:34 2022

@author: ma1012017
"""

from pymavlink import mavutil
from sys import platform
import pickle
import time


UDP = True
GROSS = False


def request_message_interval(message_id: int, frequency_hz: float):
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
    master = mavutil.mavlink_connection("udp:127.0.0.1:3000", baud=1000000)
elif GROSS:
    master = mavutil.mavlink_connection("///dev/ttyACM0")
else:
    master = mavutil.mavlink_connection("/dev/serial0", baud=1000000)
    
master.wait_heartbeat()
print("Connected")
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 1000)
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 1000)
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 100)
print("changed message interval")

i = 0
last_thrust = 0
current_seq = 0
save = False
finished = False

with open(time.strftime("%H%M%S.pickle"), 'wb') as f:
    while not finished:
        try:
            msg = master.recv_match(type=['ATTITUDE', 'RC_CHANNELS', 'SERVO_OUTPUT_RAW', 'MISSION_CURRENT'])
            if msg is not None:
                if msg.get_type() == "MISSION_CURRENT":
                    current_seq = msg.seq
                else:
                    if msg.get_type() == "SERVO_OUTPUT_RAW":
                        last_thrust = (msg.servo3_raw-1000)/1000
                    elif msg.get_type() == "ATTITUDE":
                        if msg.roll > 0.2:
                            finished = True
                    elif msg.get_type() == "RC_CHANNELS":
                        if abs(msg.chan3_raw-1500) > 100:
                            finished = True
                            master.set_mode_px4("MISSION", None, None)
                            print("aborted due to manual input")
                    if save:
                        pickle.dump(msg, f)
                        i += 1
                        if i > 100:
                            print("saving...")
                            i = 0
            if current_seq == 6:
                offboard = True
                master.mav.set_attitude_target_send(
                    int(time.time()), master.target_system,
                    master.target_component,
                    0b010000000, # ignore throttle and attitude
                    [0,0,0,0], # no quaternions
                    0.5, 0, 0, # body roll 
                    last_thrust, # thrust
                    [0,0,0]) # no 3D thrust
                master.set_mode_px4("OFFBOARD", None, None)
                if not save:
                    save = True
                    print("start saving")
            if finished:
                master.waypoint_set_current_send(7)
                print("set to loiter")
                master.set_mode_px4("MISSION", None, None)
                print("finished")
        except KeyboardInterrupt:
            master.set_mode_px4("MISSION", None, None)
            print("Program stopped")
            break

master.close()