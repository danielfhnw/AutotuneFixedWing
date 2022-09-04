# -*- coding: utf-8 -*-

import numpy as np
import time
import matplotlib.pyplot as plt


def simulate(ff, p, i, w1, w2):
    uk = 0
    uk1 = 0
    uk2 = 0
    yk = 0
    yk1 = 0
    yk2 = 0
    soll = 1
    integ = 0
    j = 0

    for n in range(300):
        e = soll-yk
        uk = np.clip(soll*ff + e*p + integ*i, -1, 1)
        yk = yk1*coefs[0] + yk2*coefs[1] + uk*coefs[2] + uk1*coefs[3] + uk2*coefs[4]
        if n > 200:
            j += w2*e*e
            yk += 0.2
        else:
            j += w1*e*e
        integ = integ+e
        yk2 = yk1
        yk1 = yk
        uk2 = uk1
        uk1 = uk
    
    return j
    

coefs = [1.50795676, -0.61949492, 0.20847013, -0.54415582, 0.76036029] 

ff_list = [0.4, 0.8, 1.6, 3.2, 6.4]
p_list = [0, 0.04, 0.08, 0.16, 0.32, 0.64]
i_list = [0, 0.04, 0.08, 0.16, 0.32]

w1 = 1
w2 = 1

j = np.zeros([len(ff_list), len(p_list), len(i_list)])

for k,ff in enumerate(ff_list):
    for l,p in enumerate(p_list):
        for m,i in enumerate(i_list):
            j[k][l][m] = simulate(ff,p,i,w1,w2)


fig, axs = plt.subplots(2,2)
X,Y = np.meshgrid(i_list, p_list)
for o in range(4):
    Z = j[o]
    if o == 0:
        plt.sca(axs[0,0])
    elif o == 1:
        plt.sca(axs[0,1])
    elif o == 2:
        plt.sca(axs[1,0])
    elif o == 3:
        plt.sca(axs[1,1])
    plt.contour(X,Y,Z)
    plt.colorbar(plt.contourf(X, Y, Z))
    minim = np.where(Z == np.min(Z))
    plt.plot(X[0][minim[1][0]], Y[minim[0][0]][0], 'ro', ms=10)
    plt.text(X[0][minim[1][0]] + 0.01, Y[minim[0][0]][0], str(np.min(Z)), 
             fontsize=16, backgroundcolor=(1.0, 1.0, 1.0))
    plt.title(f"FF = {ff_list[o]}")
    plt.xlabel("I")
    plt.ylabel("P")
plt.show()


ff_index = np.where(j == np.min(j))[0][0]
p_index = np.where(j == np.min(j))[1][0]
i_index = np.where(j == np.min(j))[2][0]
print(f"Optimale Parameter bei ff: {ff_list[ff_index]}, p: {p_list[p_index]}, i:{i_list[i_index]}")
            
#%% Einzelsimulation

import numpy as np
import time
import matplotlib.pyplot as plt

coefs = [1.50795676, -0.61949492, 0.20847013, -0.54415582, 0.76036029]

ff = 0.4
p = 0.16
i = 0.04

uk = 0
uk1 = 0
uk2 = 0
yk = 0
yk1 = 0
yk2 = 0
soll = 1
integ = 0
j = 0

y_step = [0]
u_step = [0]

for n in range(300):
    e = soll-yk
    uk = np.clip(soll*ff + e*p + integ*i, -1, 1)
    yk = yk1*coefs[0] + yk2*coefs[1] + uk*coefs[2] + uk1*coefs[3] + uk2*coefs[4]
    j += e*e
    if n > 200:
        yk += 0.2
    integ = integ+e
    yk2 = yk1
    yk1 = yk
    uk2 = uk1
    uk1 = uk
    y_step.append(yk)
    u_step.append(uk)




plt.figure(5)
plt.title("Simulationsergebnisse")
plt.xlabel("Zeitschritte")
plt.ylabel("Drehgeschwindigkeit [rad/s]")
plt.grid(True)
plt.plot(y_step)
plt.plot(soll*np.ones(len(y_step)))