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
    return np.array([x[0], x[1],x[2],x[3],1.,x[1]**2,x[2]**2,x[3]**2,x[1]*x[2],x[1]*x[3],x[2]*x[3]])


def psi_u(u):
    """Compute input basis functions
    Args:
        u: input observables
    """
    return np.array([u[0], u[1]])


camera_x = []


with open('06_04_x_cam_a.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        camera_x.append(float(row[0]))
camera_x = camera_x[100:]       
odom_x= []

with open('06_04_x_odom_a.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        odom_x.append(float(row[0]))
odom_x = odom_x[620:]

dt = 1/310 #280
T = 12.
steps = int(T/dt)
horizon = 3300
t_array = np.linspace(0,horizon-1,horizon)*dt

x1 = np.zeros( (4,steps) )


theta_l = np.zeros(steps)
theta_r = np.zeros(steps)

pol = -1

for i in range(0,steps-1):
    if i%921==0:
        pol*=-1
    theta_l[i+1] = theta_l[i] + 0.0098*pol #0.0108
    theta_r[i+1] = theta_r[i] + 0.0098*pol
    
u = np.array([theta_l,theta_r])

K_koop = np.array([[ 9.57300327e-01,  2.87666739e-03,  7.19845458e-05,
         7.19845461e-05,  3.00828301e-05, -1.44802375e-03,
        -1.66039396e-05, -1.66039396e-05, -1.53978515e-04,
        -1.53978515e-04, -1.66039398e-05, -2.56728285e-05,
         1.48805084e-03],
       [-3.58206989e-02,  9.70360192e-01,  5.13600713e-04,
         5.13600713e-04, -7.00691412e-05, -4.20241087e-03,
        -4.60464235e-05, -4.60464235e-05, -7.08199275e-04,
        -7.08199275e-04, -4.60464234e-05,  1.01643988e-03,
         4.88534594e-04],
       [-1.45641989e-01,  3.68737526e-02,  4.97841934e-01,
         4.97841934e-01,  7.95902484e-03, -7.24209602e-02,
         2.95133400e-04,  2.95133400e-04,  1.50735247e-03,
         1.50735247e-03,  2.95133400e-04, -6.55145845e-02,
         6.63365937e-02],
       [-1.45641989e-01,  3.68737526e-02,  4.97841934e-01,
         4.97841934e-01,  7.95902484e-03, -7.24209602e-02,
         2.95133400e-04,  2.95133400e-04,  1.50735247e-03,
         1.50735247e-03,  2.95133400e-04, -6.55145845e-02,
         6.63365937e-02]])

for i in range(0,steps-1):
    print('i: ', i)
    x1[:,i+1] = K_koop@np.concatenate( (psi_x(x1[:,i]), psi_u(u[:,i])  ) )
   

plt.plot(t_array,x1[0,:horizon], label="Koopman")
plt.title('x position 0.1 m/s ')
plt.plot(t_array,(camera_x[:horizon]), label="Realsense")
plt.plot(t_array,(odom_x[:horizon]), label="Odom")

plt.legend(loc="upper left")
plt.ylabel('meters')
plt.xlabel('Time(sec)')
plt.show()

plt.plot(t_array,x1[1,:horizon],'.', label="Koopman")
plt.title('x velocity 0.1 m/s')
plt.ylabel('rad')
plt.xlabel('Time(sec)')
plt.show()



plt.plot(t_array,x1[2,:horizon],'.', label="Koopman")
plt.title('left velocity 0.1 m/s')
plt.legend(loc="upper left")
plt.ylabel('rad/s')
plt.xlabel('Time(sec)')
plt.show()

plt.plot(t_array,x1[3,:horizon],'.', label="Koopman")
plt.title('right velocity 0.1 m/s')
plt.legend(loc="upper left")
plt.ylabel('rad/s')
plt.xlabel('Time(sec)')
plt.show()