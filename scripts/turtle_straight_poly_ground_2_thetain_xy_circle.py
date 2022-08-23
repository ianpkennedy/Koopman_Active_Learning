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
    return np.array([x[0], x[1],x[2],x[3],x[4],x[5], 1.,x[2]**2,x[3]**2,x[4]**2,x[5]**2, x[2]*x[4],x[2]*x[5],x[3]*x[4],x[3]*x[5],x[4]*x[5],x[2]*x[3],x[0]**2,x[1]**2,x[0]*x[1],x[0]*x[4],x[0]*x[5],x[1]*x[4],x[1]*x[5], (x[0]**2)*(x[1]**2 ) ])


def psi_u(u):
    """Compute input basis functions
    Args:
        u: input observables
    """  
    return np.array([u[0], u[1]])


# camera_x = []


# with open('06_02_x_data_theta.csv','r',) as file:
#     reader = csv.reader(file,delimiter = '\t')
#     for row in reader:
#         camera_x.append(float(row[0]))

# camera_y = []


# with open('06_02_y_data_theta.csv','r',) as file:
#     reader = csv.reader(file,delimiter = '\t')
#     for row in reader:
#         camera_y.append(float(row[0]))

camera_x = []


with open('06_08_x.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        camera_x.append(float(row[0]))
camera_x = camera_x[2100:]

camera_y = []


with open('06_08_y.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        camera_y.append(float(row[0]))
camera_y = camera_y[3000:]


odom_x= []

with open('06_08_x_odom.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        odom_x.append(float(row[0]))
odom_x = odom_x[1700:]


odom_y= []

with open('06_08_y_odom.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        odom_y.append(float(row[0]))
odom_y = odom_y[2500:]


dt = 1/310
T =20.
steps = int(T/dt)
horizon = 5000
t_array = np.linspace(0,horizon-1,horizon)*dt

x1 = np.zeros( (6,steps) )


# theta_l = np.arange(0.,200.,0.02)
# theta_r = np.arange(0.,200.,0.02)

# theta_l = 1*np.arange(0,100,0.006)
# theta_r = 1*np.arange(0,200,0.014)



theta_l = 1*np.arange(0,100,0.0078)
theta_r = 1*np.arange(0,200,0.0117)

theta_l = theta_l[0:921]
theta_r = theta_r[0:921]

theta_l_tot = np.hstack((theta_l,np.flip(theta_l)))

theta_r_tot = np.hstack((theta_r,np.flip(theta_r)))

theta_l_tot = np.hstack((theta_l_tot,theta_l_tot))
theta_r_tot = np.hstack((theta_r_tot,theta_r_tot))
theta_l_tot = np.hstack((theta_l_tot,theta_l_tot))
theta_r_tot = np.hstack((theta_r_tot,theta_r_tot))

u = np.array([theta_l_tot,theta_r_tot])

K_koop = np.array([[ 9.66846467e-01, -3.48963062e-02,  1.26995048e-03,
         3.15256821e-03,  5.06304419e-04, -2.44348629e-04,
         2.22554350e-07, -1.11286656e-02,  4.78734759e-02,
        -4.81591805e-04,  1.59000762e-02,  8.64168295e-04,
        -6.62396746e-05, -4.71524312e-04, -9.08943593e-04,
        -2.33558480e-02, -1.74700565e-02,  1.30927842e-01,
         1.05974702e+00, -6.28161739e-01,  5.34023923e-04,
         7.64350909e-04, -1.27794858e-03, -1.86298935e-03,
        -3.99903720e+00,  1.60458478e-03,  2.14270477e-05],
       [-1.11790654e-02,  9.63018404e-01,  7.72717189e-04,
         9.70046602e-04, -7.50138161e-06,  3.50667490e-05,
        -8.85581617e-06, -4.88696057e-03,  2.03969635e-02,
         2.10566913e-05, -7.70489236e-04,  1.07401807e-05,
         8.40588848e-05, -4.31161811e-04, -6.37546052e-04,
         1.13397316e-03,  3.90111663e-03,  3.18787564e-02,
         2.18474607e-01, -6.05841473e-02,  2.73295032e-04,
         4.11582852e-04, -4.30597524e-04, -6.48357607e-04,
        -1.35194072e+00,  8.33262878e-04, -3.95201131e-05],
       [ 9.40564707e-02,  3.93091353e-02,  9.53439905e-01,
         9.75431220e-03,  3.51717189e-04,  4.38980414e-04,
         1.18888996e-04, -2.91240515e-02,  4.38698710e-01,
        -6.59041345e-05,  1.38491363e-03,  1.89223218e-03,
         2.71536523e-03, -3.69139167e-03, -5.55530767e-03,
        -2.10706152e-03, -2.11755273e-01, -8.74766776e-01,
        -3.02481447e+00,  2.61242229e+00,  2.73450286e-03,
         4.09839173e-03, -6.39157655e-03, -9.58255572e-03,
         7.29560896e+00, -2.84466118e-02,  1.76799460e-02],
       [ 3.48484607e-02,  4.60978041e-02,  6.83146619e-03,
         9.43233920e-01, -9.03166494e-05, -2.50620509e-05,
        -1.18148461e-04,  7.06089576e-03,  3.79185271e-02,
         6.78502151e-05, -1.70118695e-03, -9.97795521e-04,
        -1.34623500e-03, -2.38326219e-03, -3.55277236e-03,
         2.61849566e-03,  1.16427978e-01, -1.24468438e-01,
         6.67227959e-01, -1.69377112e-01,  2.51071317e-03,
         3.77011124e-03, -1.77391202e-03, -2.66676705e-03,
        -3.18774903e+00, -1.08849540e-02,  6.43926214e-03],
       [-1.03484336e+00,  1.51521606e+00, -5.16672069e-02,
         1.63369681e-01,  3.00736794e-01,  4.54333525e-01,
         7.17243805e-03, -9.76338296e-02,  1.08241308e+00,
         2.26040558e-03, -4.81258340e-02,  1.03615950e-03,
         5.89122844e-03,  6.66927515e-03,  1.06384366e-02,
         7.28103629e-02, -1.02183789e+00,  6.51091943e+00,
        -1.69232177e+01, -2.34269830e+00,  1.24674588e-01,
         1.87127017e-01, -2.54966544e-01, -3.82620780e-01,
        -1.33027722e+01, -3.70085465e-01,  2.39922908e-01],
       [-1.55226503e+00,  2.27282408e+00, -7.75008103e-02,
         2.45054521e-01,  4.50465800e-01,  6.81934183e-01,
         1.07586571e-02, -1.46450744e-01,  1.62361961e+00,
         4.08258255e-03, -9.59840893e-02,  5.71994258e-04,
         9.46623885e-03,  9.87059590e-03,  1.60509818e-02,
         1.48434454e-01, -1.53275684e+00,  9.76637914e+00,
        -2.53848265e+01, -3.51404744e+00,  1.86990601e-01,
         2.80705508e-01, -3.82415299e-01, -5.73952751e-01,
        -1.99541583e+01, -5.55128197e-01,  3.59884362e-01]])



for i in range(0,steps-1):
   print('i: ', i)
   x1[:,i+1] = K_koop@np.concatenate( (psi_x(x1[:,i]), psi_u(u[:,i])  ) )
   


plt.plot(t_array,x1[0,:horizon], label="Koopman")
# plt.title('x position 0.1 m/s ')
plt.plot(t_array,(camera_x[:horizon]), label="Realsense")
plt.plot(t_array,(odom_x[:horizon]), label="Odom")

plt.legend(loc="upper left")
plt.ylabel('meters')
plt.xlabel('Time(sec)')
plt.show()

plt.plot(t_array,x1[1,:horizon], label="Koopman")
# plt.title('y position 0.1 m/s ')
plt.plot(t_array,(camera_y[:horizon]), label="Realsense")
plt.plot(t_array,(odom_y[:horizon]), label="Odom")

plt.legend(loc="upper left")
plt.ylabel('meters')
plt.xlabel('Time(sec)')
plt.show()

