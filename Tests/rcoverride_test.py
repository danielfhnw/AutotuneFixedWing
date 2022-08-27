# -*- coding: utf-8 -*-
"""
Created on Tue Aug 16 14:36:07 2022

@author: ma1012017
"""

from pymavlink import mavutil
import time


def set_servo_pwm(servo_n, microseconds):
    """ Sets AUX 'servo_n' output PWM pulse-width.

    Uses https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO

    'servo_n' is the AUX port to set (assumes port is configured as a servo).
        Valid values are 1-3 in a normal BlueROV2 setup, but can go up to 8
        depending on Pixhawk type and firmware.
    'microseconds' is the PWM pulse-width to set the output to. Commonly
        between 1100 and 1900 microseconds.

    """
    master.set_servo(servo_n, microseconds) or:
    #master.mav.command_long_send(
    #    master.target_system, master.target_component,
    #    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    #    0,            # first transmission of this command
    #    servo_n,  # servo instance, offset by 8 MAIN outputs
    #    microseconds, # PWM pulse-width
    #    0,0,0,0,0     # unused parameters
    )


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
master.mav.rc_channels_override_send(master.target_system, master.target_component, 
                                   1100, 0, 0, 0, 0, 0, 0, 0)
set_servo_pwm(3, 1200)
while time.time() - t < 10:
    pass

#master.mav.command_long_send(master.target_system, master.target_component,
#                                   mavutil.mavlink.RC_CHANNELS_OVERRIDE, 
#                                   0, 0, 0, 0, 0, 0, 0, 0)
master.mav.rc_channels_override_send(master.target_system, master.target_component, 
                                   0, 0, 0, 0, 0, 0, 0, 0)

print("stopped override")    
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)
print("Waiting for the vehicle to arm")
master.motors_disarmed_wait()
print('Disarmed!')

master.close()
