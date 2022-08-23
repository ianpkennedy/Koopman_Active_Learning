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
    # print(x)
    return np.array([x[0], x[1],x[2],x[3],x[4],x[5], 1.,x[2]**2,x[3]**2,x[4]**2,x[5]**2, x[2]*x[4],x[2]*x[5],x[3]*x[4],x[3]*x[5],x[4]*x[5],x[2]*x[3]])


def psi_u(u):
    
    return np.array([u[0], u[1]])


camera_x = []


with open('06_06_x.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        camera_x.append(0.05+float(row[0]))

camera_x = camera_x[2200:]

camera_y = []


with open('06_06_y.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        camera_y.append(0.05+float(row[0]))

camera_y = camera_y[3200:]

odom_x= []

with open('06_06_x_odom.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        odom_x.append(-float(row[0]))
odom_x = odom_x[1600:]


odom_y= []

with open('06_06_y_odom.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        odom_y.append(float(row[0]))
odom_y = odom_y[2600:]

dt = 1/305
T =30.
steps = int(T/dt)
horizon = 9000
t_array = np.linspace(0,horizon-1,horizon)*dt

x1 = np.zeros( (6,steps) )

theta_l = np.zeros(steps)
theta_r = np.zeros(steps)

pol = -1

for i in range(0,steps-1):
    if i%921==0:
        pol*=-1
    theta_l[i+1] = theta_l[i] + 0.01*pol
    theta_r[i+1] = theta_r[i] + 0.01*pol
    
u = np.array([theta_l,theta_r])

K_koop = np.array([[ 8.95244458e-01,  3.27864221e-02,  4.21020221e-03,
        -6.94911504e-04,  3.64624843e-05,  3.64624843e-05,
         5.11407785e-04, -6.47754048e-03, -2.52285250e-02,
        -3.09613282e-05, -3.09613282e-05, -1.97737276e-04,
        -1.97737276e-04, -3.81575891e-04, -3.81575891e-04,
        -3.09613282e-05,  2.44058970e-02, -1.20154487e-02,
         1.40240001e-02],
       [-2.09602600e-02,  9.70673991e-01,  1.61922160e-03,
         2.34139393e-03,  2.90553495e-05,  2.90553495e-05,
         2.44215806e-05, -1.67957170e-03, -9.94962010e-03,
        -4.54386187e-06, -4.54386187e-06, -8.94754906e-05,
        -8.94754906e-05,  1.14051466e-04,  1.14051466e-04,
        -4.54386187e-06,  7.35147486e-03,  3.00704723e-04,
         9.01731039e-04],
       [-3.42661445e-01,  2.69754800e-01,  9.61666551e-01,
        -1.32493860e-02,  6.01662706e-04,  6.01662706e-04,
         4.51028122e-03,  1.68505843e-02,  3.04491235e-01,
        -2.94182948e-04, -2.94182948e-04, -3.19570986e-03,
        -3.19570986e-03, -2.96279992e-03, -2.96279992e-03,
        -2.94182948e-04,  3.59830435e-01, -6.45979530e-02,
         6.77969666e-02],
       [-1.01987739e-03,  2.92797882e-02,  2.66492890e-03,
         9.68480300e-01,  2.66731746e-04,  2.66731746e-04,
        -6.36308851e-04, -1.36543917e-02, -1.99273607e-02,
        -1.61204828e-05, -1.61204828e-05, -3.24689796e-04,
        -3.24689796e-04,  4.09902331e-04,  4.09902331e-04,
        -1.61204828e-05,  1.59850039e-02,  3.45429972e-02,
        -3.50206718e-02],
       [-1.19402796e+00,  2.25331724e+00, -1.76284053e-02,
        -5.95570130e-02,  4.97943313e-01,  4.97943313e-01,
         9.37728340e-03, -2.61115522e-01,  2.60665270e-01,
         9.12910832e-06,  9.12910832e-06, -1.12666087e-03,
        -1.12666087e-03, -9.93269663e-03, -9.93269663e-03,
         9.12910832e-06,  3.02470833e-01, -1.75150593e-01,
         1.52281353e-01],
       [-1.19402796e+00,  2.25331724e+00, -1.76284053e-02,
        -5.95570130e-02,  4.97943313e-01,  4.97943313e-01,
         9.37728340e-03, -2.61115522e-01,  2.60665270e-01,
         9.12910832e-06,  9.12910832e-06, -1.12666087e-03,
        -1.12666087e-03, -9.93269663e-03, -9.93269663e-03,
         9.12910832e-06,  3.02470833e-01, -1.75150593e-01,
         1.52281353e-01]])




for i in range(0,steps-1):
   # print('i: ', i)
   x1[:,i+1] = K_koop@np.concatenate( (psi_x(x1[:,i]), psi_u(u[:,i])  ) )

plt.plot(t_array,x1[0,:horizon], label="Koopman")
plt.title('x position 0.1 m/s ')
plt.plot(t_array,(camera_x[:horizon]), label="Realsense")
plt.plot(t_array,(odom_x[:horizon]), label="Odom")
plt.legend(loc="upper left")
plt.ylabel('meters')
plt.xlabel('Time(sec)')
plt.show()

plt.plot(t_array,x1[1,:horizon], label="Koopman")
plt.title('y position 0.1 m/s ')
plt.plot(t_array,(camera_y[:horizon]), label="Realsense")
plt.plot(t_array,(odom_y[:horizon]), label="Odom")
plt.legend(loc="upper left")
plt.ylabel('meters')
plt.xlabel('Time(sec)')
plt.show()

# plt.plot(t_array,x1[2,:horizon],'.', label="Koopman")
# plt.title('x velocity 0.1 m/s')
# # plt.legend(loc="upper left")
# plt.ylabel('rad')
# plt.xlabel('Time(sec)')
# plt.show()

# plt.plot(t_array,x1[3,:horizon],'.', label="Koopman")
# plt.title('y velocity 0.1 m/s')
# # plt.legend(loc="upper left")
# plt.ylabel('rad')
# plt.xlabel('Time(sec)')
# plt.show()

# plt.plot(t_array,x1[4,:horizon],'.', label="Koopman")
# plt.title('left wheel velocity 0.1 m/s')
# # plt.plot((theta_lv[0:horizon]), label="Turtlebot")
# plt.legend(loc="upper left")
# plt.ylabel('rad/s')
# plt.xlabel('Time(sec)')
# # plt.ylim([0, 3.5])
# plt.show()

# plt.plot(t_array,x1[5,:horizon],'.', label="Koopman")
# plt.title('right wheel velocity 0.1 m/s')
# # plt.plot((theta_rv[0:horizon]), label="Turtlebot")
# plt.legend(loc="upper left")
# plt.ylabel('rad/s')
# plt.xlabel('Time(sec)')
# # plt.ylim([0, 3.5])
# plt.show()