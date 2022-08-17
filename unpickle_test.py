# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 16:16:14 2022

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

with open("log.pickle", "rb") as fp:
    for msg in pickleLoader(fp):
        if msg.get_type() == "ATTITUDE":
            roll.append(msg.roll)
            time.append(msg.time_boot_ms)
        

plt.plot(time,roll,'.')
plt.grid(True)

#%% 

diff = time[-1] - time[0]
print(len(roll)/(diff/1000))