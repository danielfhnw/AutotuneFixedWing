# -*- coding: utf-8 -*-

import pickle
import numpy as np
from matplotlib import pyplot as plt
from sklearn.linear_model import LinearRegression


def pickleLoader(pklFile):
    try:
        while True:
            yield pickle.load(pklFile)
    except EOFError:
        pass

lower_t = 155500
upper_t = 157300
dt = 20
axis = "ROLL"

velocity = []
time = []
servo = []
time2 = []

with open("Logs/160811.pickle", "rb") as fp:
    for msg in pickleLoader(fp):
        if msg.get_type() == "ATTITUDE":
            if axis == "ROLL":    
                velocity.append(msg.rollspeed)
            elif axis == "PITCH":
                velocity.append(msg.pitchspeed)
            elif axis == "YAW":
                velocity.append(msg.yawspeed)
            time.append(msg.time_boot_ms)
        elif msg.get_type() == "SERVO_OUTPUT_RAW":
            if axis == "ROLL":
                servo.append(msg.servo1_raw)
            elif axis == "PITCH":
                servo.append(msg.servo3_raw)
            elif axis == "YAW":
                servo.append(msg.servo4_raw)
            time2.append(msg.time_usec)
            
plt.figure(0)
plt.plot(time,velocity,'.')
plt.plot(time2,servo,'.')
plt.grid(True)
plt.xlim([lower_t, upper_t])
plt.ylim([-3, 2])
plt.show()
plt.figure(1)
plt.plot(time,velocity,'.')
plt.plot(time2,servo,'.')
plt.grid(True)
plt.xlim([lower_t*1000, upper_t*1000])
plt.ylim([0, 2000])
plt.show()
            
#%% Skalierung

for i,(t,s) in enumerate(zip(time2,servo)):
    time2[i] = t/1000
    servo[i] = (s-1500)/(-500)

plt.figure(2)
plt.plot(time,velocity,'.')
plt.plot(time2,servo,'.')
plt.grid(True)
plt.xlim([lower_t, upper_t])
plt.ylim([-3, 2])
plt.show()

#%% Resampling

time_res = np.arange(lower_t, upper_t, dt)
velocity_res = np.interp(time_res, time, velocity)
servo_res = np.interp(time_res, time2, servo)

plt.figure(3)
plt.plot(time_res,velocity_res,'.')
plt.plot(time_res,servo_res,'.')
plt.grid(True)
plt.ylim([-3, 2])
plt.show()

#%% ARX Modell

velocity_res = np.expand_dims(velocity_res, axis=1)
servo_res = np.expand_dims(servo_res, axis=1)

y = velocity_res[2:]

X = np.concatenate([velocity_res[1:-1], velocity_res[:-2], servo_res[2:], servo_res[1:-1], servo_res[:-2]], axis=1)
#X = np.concatenate([velocity_res[1:-1], servo_res[2:]], axis=1)

lr = LinearRegression().fit(X, y)

print(lr.coef_)

y_pred = lr.predict(X)

plt.figure(4)
plt.plot(time_res[2:],y,'-')
plt.plot(time_res[2:],y_pred,'*')
plt.grid(True)
plt.ylim([-3, 2])
plt.show()
