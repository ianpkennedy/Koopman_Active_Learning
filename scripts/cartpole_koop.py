import matplotlib.pyplot as plt
import numpy as np

import csv




theta = []


with open('04_28theta_a.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        theta.append(float(row[0]))

thetadot = []


with open('04_28thetadot_a.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        thetadot.append(float(row[0]))


x = []


with open('04_28x_a.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        x.append(float(row[0]))


xdot = []


with open('04_28xdot_a.csv','r',) as file:
    reader = csv.reader(file,delimiter = '\t')
    for row in reader:
        xdot.append(float(row[0]))

def psi_x(x):
    
    """Compute state basis functions
    Args:
        x: state observables
    """
    return np.array([x[0], x[1], x[2], x[3], np.sin(x[2]), np.cos(x[2]),1])

def psi_u(u, theta):
    """Compute input basis functions
    Args:
        u: input observables
    """
    
    return np.array([u, u*np.cos(theta)])



T = 10
steps = 10000
dt = T/steps
t_array = np.linspace(0,T,steps)
u = np.zeros(steps)

K_koop = np.array([[ 9.99999996e-01,  9.99840071e-04,  9.30620166e-06,
         2.36187493e-08, -8.76284872e-06, -3.28481007e-09,
         3.28474881e-09,  0.00000000e+00,  0.00000000e+00],
       [-3.45636073e-07,  9.99781604e-01, -2.10544086e-02,
         3.17593648e-05,  2.48138753e-02,  3.31324419e-06,
        -3.31344521e-06,  0.00000000e+00,  0.00000000e+00],
       [-6.94237031e+00,  3.12019318e-05,  9.99216678e-01,
        -1.00454108e-03, -1.10992455e+00, -1.29538330e-06,
         7.85594295e-01,  0.00000000e+00,  0.00000000e+00],
       [ 2.26916076e-04, -1.50665143e-03, -3.04477109e-02,
         1.00022878e+00,  4.66607002e-02,  3.39241389e-04,
        -3.39241595e-04,  0.00000000e+00,  0.00000000e+00]])

x1 = np.zeros( (4,steps) )
x1[2,0] = np.pi/4


for i in range(steps):
    u[i]=0


for i in range(0,steps-1):
    x1[:,i+1] = K_koop@np.concatenate( (psi_x(x1[:,i]), psi_u(u[i],x1[2,i])  ) )

horizon = 10000
plt.plot(x1[0,:horizon],'.', label="Koopman")
plt.title('x ')
plt.plot((x[0:horizon]), label="Gazebo")
plt.legend(loc="upper left")
plt.ylabel('m')
plt.xlabel('Timesteps (0.001s)')
plt.show()

plt.plot(x1[1,:horizon],'.', label="Koopman")
plt.title('xdot ')
plt.plot((xdot[0:horizon]), label="Gazebo")
plt.legend(loc="upper left")
plt.ylabel('m/s')
plt.xlabel('Timesteps (0.001s)')
plt.show()



plt.plot(x1[2,:horizon],'.', label="Koopman")
plt.title('theta ')
plt.plot(theta[0:horizon], label="Gazebo")
plt.legend(loc="lower left")
plt.ylabel('rad')
plt.xlabel('Timesteps (0.001s)')
plt.show()


plt.plot(x1[3,:horizon], '.', label="Koopman")
plt.title('thetadot ')
plt.plot((thetadot[0:horizon]), label="Gazebo")
plt.legend(loc="lower left")
plt.ylabel('rad/s')
plt.xlabel('Timesteps (0.001s)')
plt.show()

