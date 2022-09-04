# -*- coding: utf-8 -*-

from pymavlink import mavutil
from sys import platform
import pickle
import time
import math


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
    master = mavutil.mavlink_connection("/dev/serial0", baud=500000)
    
master.wait_heartbeat()
print("Connected")
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 10)
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 10)
print("reset message interval")


i = 0
steps_i = 0
steps = [[0.5, 0, 0],
         [-0.5, 0, 0],
         [0, 0.5, 0],
         [0, -0.5, 0],
         [0, 0, 0.5],
         [0, 0, -0.5]]
last_thrust = 0
current_seq = 0
finished = False
lasttime = time.time()
starttime = 0
finishedtime = 0
interval_change = True

with open(time.strftime("%H%M%S.pickle"), 'wb') as f:
    while True:
        try:
            msg = master.recv_match(type=['ATTITUDE', 'RC_CHANNELS', 'SERVO_OUTPUT_RAW', 'MISSION_CURRENT'])
            if msg is not None:
                if msg.get_type() == "MISSION_CURRENT":
                    current_seq = msg.seq
                elif msg.get_type() == "SERVO_OUTPUT_RAW":
                    last_thrust = (msg.servo3_raw-1000)/1000
                elif msg.get_type() == "ATTITUDE":
                    pass
                elif msg.get_type() == "RC_CHANNELS":
                    if abs(msg.chan4_raw-1500) > 250 and finishedtime == 0:
                        finished = True
                        print("aborted due to manual input")
                pickle.dump(msg, f)
                i += 1
                if i > 100:
                    print("saving...")
                    i = 0
                        
            if current_seq == 5 and interval_change:
                request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 1000)
                request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 1000)
                print("changed message interval")
                interval_change = False
            elif current_seq == 6:
                if starttime == 0:
                    starttime = time.time()
                if time.time() - starttime > 1:
                    master.set_mode_px4("MISSION", None, None)
                else:
                    master.set_mode_px4("OFFBOARD", None, None)
            elif current_seq == 7 and not interval_change:
                if steps_i < len(steps)-1:
                    steps_i += 1
                    print(f"rates setpoint for next round: {steps[steps_i]}")
                    starttime = 0
                else:
                    finished = True
                request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)
                request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 10)
                print("reset message interval")
                interval_change = True
            
            if time.time()-lasttime > 0.1:
                lasttime = time.time()
                master.mav.set_attitude_target_send(
                    int(time.time()), master.target_system,
                    master.target_component,
                    0b010000000, # ignore attitude
                    [0,0,0,0], # no quaternions
                    *steps[steps_i], # angular rates setpoints 
                    last_thrust, # thrust
                    [0,0,0]) # no 3D thrust
            
            if finished:
                lasttime = math.inf
                master.waypoint_set_current_send(9)
                print("set to loiter")
                master.set_mode_px4("MISSION", None, None)
                print("waiting for storage...")
                finishedtime = time.time()
                finished = False
                
            if finishedtime != 0 and time.time() - finishedtime > 5:
                    print("finished")
                    break
                
        except KeyboardInterrupt:
            master.waypoint_set_current_send(9)
            master.set_mode_px4("MISSION", None, None)
            print("Program stopped due to keyboard interrupt")
            break

master.close()