# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 22:21:17 2022

@author: ma1012017
"""

import pickle
from matplotlib import pyplot as plt


def pickleLoader(pklFile):
    try:
        while True:
            yield pickle.load(pklFile)
    except EOFError:
        pass


roll = []
time = []
servo = []
time2 = []
types = []

with open("Logs/identlog5.pickle", "rb") as fp:
    for msg in pickleLoader(fp):
        if msg.get_type() == "ATTITUDE":
            roll.append(msg.roll)
            time.append(msg.time_boot_ms)
        elif msg.get_type() == "SERVO_OUTPUT_RAW":
            servo.append(msg.servo1_raw)
            time2.append(msg.time_usec)
        types.append(msg.get_type())
            
plt.figure(1)
plt.plot(time,roll,'.')
plt.grid(True)
plt.show()

plt.figure(2)
plt.plot(time2,servo,'.')
plt.grid(True)
plt.show()

#%% 

diff = time[-1] - time[0]
print(len(roll)/(diff/1000))

diff = time2[-1] - time2[0]
print(len(servo)/(diff/1000000))