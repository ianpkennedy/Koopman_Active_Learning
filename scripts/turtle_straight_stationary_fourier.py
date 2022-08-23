#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 28 16:29:43 2022

@author: iankennedy
"""

import csv
import numpy as np
import matplotlib.pyplot as plt


def psi_x(x):
    """Compute state basis functions
    Args:
        x: state observables
    """
    print(x)
    return np.array([x[0], x[1],x[2],x[3], 1., np.sin(x[0]), np.cos(x[0]), 1., np.sin(x[1]), np.cos(x[1]) ])

def psi_u(u):
    """Compute input basis functions
    Args:
        u: input observables
    """
    
    return np.array([u[0], u[1]])


theta_l = []


with open('05_02_l_data_ang.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        theta_l.append(float(row[0]))


theta_r = []


with open('05_02_r_data_ang.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        theta_r.append(float(row[0]))


encoder_to_rad = 0.001535
dt = 1/307
T = 32.
steps = int(T/dt)
horizon = 4000

x1 = np.zeros( (4,steps) )
x1[2,0] = 3.024
x1[3,0] = 3.024

u = -3.0303*np.ones( (2,steps) )


K_koop = np.array([[ 1.18591512e+00, -1.85891011e-01,  7.21816448e+00,
         7.21816574e+00, -2.18701247e+01, -1.31982812e-04,
         6.64025274e-03, -2.18701422e+01,  9.90410306e-04,
        -8.19941147e-03, -7.22930096e-06, -7.62779712e-06],
       [ 1.96891006e-01,  8.03134445e-01,  6.97830592e+00,
         6.97830163e+00, -2.11434055e+01,  2.18142293e-04,
         6.19535181e-03, -2.11434183e+01,  6.80020812e-04,
        -7.67826059e-03, -5.02982031e-06, -5.41546242e-06],
       [ 6.94861889e-04, -6.94773693e-04,  4.43125560e-01,
         4.43125541e-01,  1.72335830e-01,  1.70558415e-05,
         1.16890137e-05,  1.72335852e-01, -1.32855402e-05,
        -1.61219325e-05,  5.23251864e-07,  5.23678855e-07],
       [ 6.94861889e-04, -6.94773693e-04,  4.43125560e-01,
         4.43125541e-01,  1.72335830e-01,  1.70558415e-05,
         1.16890137e-05,  1.72335852e-01, -1.32855402e-05,
        -1.61219325e-05,  5.23251864e-07,  5.23678855e-07]])



for i in range(0,steps-1):
   x1[:,i+1] = K_koop@np.concatenate( (psi_x(x1[:,i]), psi_u(u[:,i])  ) )
   
   
plt.plot(x1[0,:horizon],'.', label="Koopman")
plt.title('left ')
plt.plot((theta_l[0:horizon]), label="Turtlebot")
plt.legend(loc="upper left")
plt.ylabel('rad')
plt.xlabel('Timesteps (0.001s)')
plt.show()

plt.plot(x1[1,:horizon],'.', label="Koopman")
plt.title('right ')
plt.plot((theta_r[0:horizon]), label="Turtlebot")
plt.legend(loc="upper left")
plt.ylabel('rad')
plt.xlabel('Timesteps (0.001s)')
plt.show()

plt.plot(x1[2,:horizon],'.', label="Koopman")
plt.title('left vel')
# plt.plot((theta_lv[0:horizon]), label="Turtlebot")
plt.legend(loc="upper left")
plt.ylabel('rad/s')
plt.xlabel('Timesteps (0.001s)')
plt.ylim([0, 3.5])
plt.show()

plt.plot(x1[3,:horizon],'.', label="Koopman")
plt.title('right vel ')
# plt.plot((theta_rv[0:horizon]), label="Turtlebot")
plt.legend(loc="upper left")
plt.ylabel('rad/s')
plt.xlabel('Timesteps (0.001s)')
plt.ylim([0, 3.5])
plt.show()
