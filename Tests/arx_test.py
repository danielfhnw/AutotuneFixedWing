# -*- coding: utf-8 -*-
"""
Created on Fri Aug 26 22:24:11 2022

@author: ma1012017
"""

import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

ur = np.concatenate([100*np.ones([100,1]), np.zeros([200,1])])
up = np.concatenate([np.zeros([100,1]), 100*np.ones([100,1]), np.zeros([100,1])])
uy = np.concatenate([np.zeros([200,1]), 100*np.ones([100,1])])

yr = np.zeros(300)
yp = np.zeros([300,1])
yy = np.zeros([300,1])

a1 = 0.8
a2 = 0.1

b0 = 1
b1 = 0.2
b2 = 0.1

er = 1*np.random.rand(300,1)-0.5


for i in range(299):
    if i == 0:
        yr[i] = b0*ur[i] + er[i]
    elif i == 1:
        yr[i] = a1*yr[i-1] + b0*ur[i] + b1*ur[i-1] + er[i]
    else:
        yr[i] = a1*yr[i-1] + a2*yr[i-2] + b0*ur[i] + b1*ur[i-1] + b2*ur[i-2] + er[i]
        

yr = np.expand_dims(yr, axis=1)

plt.figure(1)
plt.plot(ur)
plt.plot(up)
plt.plot(uy)
plt.show()

plt.figure(2)
plt.plot(yr)
plt.plot(yp)
plt.plot(yy)
plt.show()

#%%

y = yr[2:]

X = np.concatenate([yr[1:-1], yr[:-2], ur[2:], ur[1:-1], ur[:-2]], axis=1)

lr = LinearRegression().fit(X, y)

print(lr.coef_)