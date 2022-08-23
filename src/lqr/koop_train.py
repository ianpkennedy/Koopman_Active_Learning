import numpy as np
import scipy.linalg as spicy_linalg

"""
Non control koopman training
"""
    
class Koop():
    def __init__(self,dt=0.0,x_state=4,u_state=1, x_curr = 0.0, theta_curr = 0.0, u_curr = 0.0, xdot_curr = 0.0, thetadot_curr = 0.0):
        """_summary_

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            x_curr (float): current x position
            theta_curr (float): current x
            u_curr (float): current cmd_vel x input
            xdot_curr (float): current linear velocity
            thetadot_curr (float): current rod velocity
            udot_curr (float): cmd_accel
            
        """
        
        self.dt = dt
        
        self.state_obs = x_state+3
        self.u_obs = u_state+1
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        self.Kcont = np.zeros([self.total_obs,self.total_obs])
        
        
        self.x_curr = x_curr
        self.xdot_curr = xdot_curr
        self.theta_curr = theta_curr
        self.thetadot_curr = thetadot_curr
        self.u_curr = u_curr
        
        self.x_old = x_curr
        self.xdot_old = xdot_curr
        self.theta_old = theta_curr
        self.thetadot_old = thetadot_curr
        self.u_old = u_curr
        
        self.divider = 1


    def train_model(self,x,xdot,theta,thetadot, u):
        
        """train model parameters using Koopman

        Args:
            x (float): cart position
            xdot (float): cart velocity
            theta (float): rod angle
            thetadot (float): rod angular acceleration
        
        Returns:
            None
        """
        
        self.x_old = self.x_curr
        self.xdot_old = self.xdot_curr
        self.theta_old = self.theta_curr
        self.thetadot_old = self.thetadot_curr
        self.u_old = self.u_curr
        
        self.x_curr = x
        self.xdot_curr = xdot
        self.theta_curr = theta
        self.thetadot_curr = thetadot
        self.u_curr = u
        
        psix_old = self.zee_x(self.x_old, self.xdot_old, self.theta_old, self.thetadot_old) #This is an array
        psix_curr = self.zee_x(self.x_curr, self.xdot_curr, self.theta_curr, self.thetadot_curr)
        
        psiu_old = self.zee_u(self.u_old, self.theta_old)
        psiu_curr = self.zee_u(self.u_curr,  self.theta_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider
        self.divider += 1
        
    def zee_x(self,x,xdot,theta,thetadot):
        """Compute state observation parameters

        Args:
            x (float): cart position
            xdot (float): cart velocity
            theta (float): rod angle
            thetadot (float): rod angular acceleration
        """
        
        return np.array([x,xdot,theta,thetadot,np.sin(theta),np.cos(theta), 1])
    
    def zee_u(self,u, theta):
        """compute control input observation parameters

        Args:
            u (float): cmd_vel command
            theta (float): rod angle
        """
        
        return np.array([u, u*np.cos(theta)])
    
    
    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
        # print('A: ', self.A)
        # print('G: ', self.G)
        # print('current divider: ', (self.cycle_count*(self.iteration_curr+1)))
        
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))
        # self.K =  np.dot(self.A/(self.cycle_count*(self.iteration_curr+1)),np.linalg.pinv(self.G/(self.cycle_count*(self.iteration_curr+1))))
        
        
        # print('K: ', self.K)
        # self.Kcont = np.real(spicy_linalg.logm(self.K))/self.dt
        K_reduced = self.K[:4,:]
        print('K reduced : ',  repr(K_reduced))
        # print('K continuous: ', self.Kcont)
        
        





















class Turtle_Koop_Vel():
    def __init__(self,dt=0.0,x_state=4,u_state=2, l_curr = 0.0, r_curr = 0.0, lv_curr = 0.0, rv_curr = 0.0, ul_curr = 0.0, ur_curr = 0.0):
        """Turtlebot Koopman training class

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            l_curr (float): current left wheel position
            r_curr (float): current right wheel position
            lv_curr(float): current levt wheel velocity
            rv_curr(float): current right wheel velocity
            ur_curr (float): current right wheel command input
            ul_curr (float): current left wheel command input
        """
        
        self.dt = dt
        
        self.state_obs = x_state+6
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        
        self.l_curr = l_curr
        self.r_curr = r_curr
        self.l_old = l_curr
        self.r_old = r_curr
        
        self.lv_curr = lv_curr
        self.rv_curr = rv_curr
        # self.lv_old = lv_curr
        # self.rv_old = rv_curr
        
        self.ul_curr = ul_curr
        self.ur_curr = ur_curr
        self.ul_old = ul_curr
        self.ur_old = ur_curr
        
        self.divider = 1


    def train_model(self,ltheta,rtheta, lvel, rvel, ul, ur):
        
        """train model parameters using Koopman

        Args:
            ltheta: updated left wheel angle
            rtheta: updated right wheel angle
            lvel: left wheel velocity
            rvel: right wheel velocity
            ul: updated left wheel command
            ur: updated right wheel command
            
        Returns:
            None
        """
        print('fourier ... ')
        self.l_old = self.l_curr
        self.r_old = self.r_curr
        self.ul_old = self.ul_curr
        self.ur_old = self.ur_curr
        self.lv_old = self.lv_curr
        self.rv_old = self.rv_curr
        
        self.l_curr = ltheta
        self.r_curr = rtheta
        self.lv_curr = lvel
        self.rv_curr = rvel
        self.ul_curr = ul
        self.ur_curr = ur
        
        psix_old = self.zee_x(self.l_old, self.r_old, self.lv_old, self.rv_old) #This is an array
        psix_curr = self.zee_x(self.l_curr, self.r_curr, self.lv_curr, self.rv_curr)
        
        psiu_old = self.zee_u(self.ul_old, self.ur_old)
        psiu_curr = self.zee_u(self.ul_curr, self.ur_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        # print(koop_curr.shape)
        # print(koop_old.shape)
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider               
             
        self.divider+=1


    def zee_x(self,ltheta, rtheta, lvel, rvel):
        """Compute state observation parameters

        Args:
            ltheta: left wheel angle
            rtheta: right wheel angle 
            ul: left wheel command
            ur: right wheel command
        """
        
        return np.array([ltheta, rtheta, lvel, rvel, 1., np.sin(ltheta),np.cos(ltheta),1., np.sin(rtheta),np.cos(rtheta)])
    
    
    def zee_u(self,ul, ur):
        """compute control input observation parameters

        Args:
            ul (float): left wheel command
            ur (float): left wheel command
        """
        
        return np.array([ul, ur])

        

    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
  
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))

        K_reduced = self.K[:4,:]
        print('K reduced : ',  repr(K_reduced))
        
 








class Turtle_Koop_Vel_X_Xdot_Mono():
    def __init__(self,dt=0.0,x_state=4,u_state=1,x_curr = 0., xdot_curr = 0., l_curr = 0.0, lv_curr = 0.0, ul_curr = 0.0):
        """Turtlebot Koopman training class

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            l_curr (float): current  wheel position
            lv_curr(float): current  wheel velocity
            ul_curr (float): current  wheel command input
        """
        
        self.dt = dt
        
        self.state_obs = x_state+3
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        
        self.l_curr = l_curr
        self.l_old = l_curr
        
        self.x_curr = x_curr
        self.x_old = x_curr
        
        self.xdot_curr = xdot_curr
        self.xdot_old = xdot_curr
        
        self.lv_curr = lv_curr
        self.lv_old = lv_curr
        
        self.ul_curr = ul_curr
        self.ul_old = ul_curr
        
        self.divider = 1


    def train_model(self, x, xdot, ltheta, lvel, ul):
        
        """train model parameters using Koopman

        Args:
            ltheta: updated left wheel angle
            lvel: left wheel velocity
            ul: updated left wheel command
            
        Returns:
            None
        """
        print('fourier ... ')
        self.l_old = self.l_curr
        self.ul_old = self.ul_curr
        self.lv_old = self.lv_curr
        self.x_old = self.x_curr
        self.xdot_old = self.xdot_curr
        
        self.l_curr = ltheta
        self.lv_curr = lvel
        self.ul_curr = ul
        self.x_curr = x
        self.xdot_curr = xdot
        
        psix_old = self.zee_x(self.x_curr, self.xdot_curr, self.l_old, self.lv_old) #This is an array
        psix_curr = self.zee_x(self.x_old, self.xdot_old, self.l_curr, self.lv_curr)
        
        psiu_old = self.zee_u(self.ul_old)
        psiu_curr = self.zee_u(self.ul_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        # print(koop_curr.shape)
        # print(koop_old.shape)
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider               
             
        self.divider+=1


    def zee_x(self,x, xdot, ltheta, lvel):
        """Compute state observation parameters

        Args:
            x: x position of the robot
            xdot: linear velocity of the robot
            ltheta: left wheel angle
            ul: left wheel command
        """
        
        return np.array([x, xdot, ltheta, lvel, 1., np.sin(ltheta),np.cos(ltheta)])
    
    
    def zee_u(self,ul):
        """compute control input observation parameters

        Args:
            ul (float):  wheel command
        """
        
        return np.array([ul])

        

    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
  
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))

        K_reduced = self.K[:4,:]
        print('K reduced : ',  repr(K_reduced))































  
class Turtle_Koop():
    def __init__(self,dt=0.0,x_state=2,u_state=2, l_curr = 0.0, r_curr = 0.0, lv_curr = 0.0, rv_curr = 0.0, ul_curr = 0.0, ur_curr = 0.0):
        """Turtlebot Koopman training class

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            l_curr (float): current left wheel position
            r_curr (float): current right wheel position
            lv_curr(float): current levt wheel velocity
            rv_curr(float): current right wheel velocity
            ur_curr (float): current right wheel command input
            ul_curr (float): current left wheel command input
        """
        
        self.dt = dt
        
        self.state_obs = x_state+6
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        
        self.l_curr = l_curr
        self.r_curr = r_curr
        self.l_old = l_curr
        self.r_old = r_curr
        
        self.lv_curr = lv_curr
        self.rv_curr = rv_curr
        # self.lv_old = lv_curr
        # self.rv_old = rv_curr
        
        self.ul_curr = ul_curr
        self.ur_curr = ur_curr
        self.ul_old = ul_curr
        self.ur_old = ur_curr
        
        self.divider = 1


    def train_model(self,ltheta,rtheta, ul, ur):
        
        """train model parameters using Koopman

        Args:
            ltheta: updated left wheel angle
            rtheta: updated right wheel angle
            lvel: left wheel velocity
            rvel: right wheel velocity
            ul: updated left wheel command
            ur: updated right wheel command
            
        Returns:
            None
        """
        print('fourier ... ')
        self.l_old = self.l_curr
        self.r_old = self.r_curr
        self.ul_old = self.ul_curr
        self.ur_old = self.ur_curr
        # self.lv_old = self.lv_curr
        # self.rv_old = self.rv_curr
        
        self.l_curr = ltheta
        self.r_curr = rtheta
        # self.lv_curr = lvel
        # self.rv_curr = rvel
        self.ul_curr = ul
        self.ur_curr = ur
        
        psix_old = self.zee_x(self.l_old, self.r_old) #This is an array
        psix_curr = self.zee_x(self.l_curr, self.r_curr)
        
        psiu_old = self.zee_u(self.ul_old, self.ur_old)
        psiu_curr = self.zee_u(self.ul_curr, self.ur_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        # print(koop_curr.shape)
        # print(koop_old.shape)
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider               
             
        self.divider+=1


    def zee_x(self,ltheta, rtheta):
        """Compute state observation parameters

        Args:
            ltheta: left wheel angle
            rtheta: right wheel angle 
            ul: left wheel command
            ur: right wheel command
        """
        
        return np.array([ltheta, rtheta, 1., np.sin(ltheta),np.cos(ltheta),1., np.sin(rtheta),np.cos(rtheta)])
    
    
    def zee_u(self,ul, ur):
        """compute control input observation parameters

        Args:
            ul (float): left wheel command
            ur (float): left wheel command
        """
        
        return np.array([ul, ur])

        

    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
  
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))

        K_reduced = self.K[:2,:]
        print('K reduced : ',  repr(K_reduced))
        
        
        
class Turtle_Koop_Poly():
    def __init__(self,dt=0.0,x_state=2,u_state=2, l_curr = 0.0, r_curr = 0.0, lv_curr = 0.0, rv_curr = 0.0, ul_curr = 0.0, ur_curr = 0.0):
        """Turtlebot Koopman training class using polynomial basis functions

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            l_curr (float): current left wheel position
            r_curr (float): current right wheel position
            lv_curr(float): current levt wheel velocity
            rv_curr(float): current right wheel velocity
            ur_curr (float): current right wheel command input
            ul_curr (float): current left wheel command input
        """
        
        self.dt = dt
        
        self.state_obs = x_state+4
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        
        self.l_curr = l_curr
        self.r_curr = r_curr
        self.l_old = l_curr
        self.r_old = r_curr
        
        self.lv_curr = lv_curr
        self.rv_curr = rv_curr
        self.lv_old = lv_curr
        self.rv_old = rv_curr
        
        self.ul_curr = ul_curr
        self.ur_curr = ur_curr
        self.ul_old = ul_curr
        self.ur_old = ur_curr
        
        self.divider = 1


    def train_model(self,ltheta,rtheta, lvel, rvel, ul, ur):
        
        """train model parameters using Koopman

        Args:
            ltheta: updated left wheel angle
            rtheta: updated right wheel angle
            lvel: left wheel velocity
            rvel: right wheel velocity
            ul: updated left wheel command
            ur: updated right wheel command
            
        Returns:
            None
        """
        
        self.l_old = self.l_curr
        self.r_old = self.r_curr
        self.ul_old = self.ul_curr
        self.ur_old = self.ur_curr
        self.lv_old = self.lv_curr
        self.rv_old = self.rv_curr
        
        self.l_curr = ltheta
        self.r_curr = rtheta
        self.lv_curr = lvel
        self.rv_curr = rvel
        self.ul_curr = ul
        self.ur_curr = ur
        
        psix_old = self.zee_x(self.l_old, self.r_old,self.lv_old,self.rv_old) #This is an array
        psix_curr = self.zee_x(self.l_curr, self.r_curr, self.lv_curr, self.rv_curr)
        
        psiu_old = self.zee_u(self.ul_old, self.ur_old)
        psiu_curr = self.zee_u(self.ul_curr, self.ur_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        # print(koop_curr.shape)
        # print(koop_old.shape)
        print('polynomial...')
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider               
             
        self.divider+=1


    def zee_x(self,ltheta, rtheta, lvel, rvel):
        """Compute state observation parameters

        Args:
            ltheta: left wheel angle
            rtheta: right wheel angle 
            ul: left wheel command
            ur: right wheel command
        """
        
        return np.array([ltheta, rtheta, lvel, rvel, 1., lvel**2, rvel**2, lvel*rvel])
    
    
    def zee_u(self,ul, ur):
        """compute control input observation parameters

        Args:
            ul (float): left wheel command
            ur (float): left wheel command
        """
        
        return np.array([ul, ur])

        

    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
  
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))

        K_reduced = self.K[:4,:]
        print('K reduced : ',  repr(K_reduced))





class Turtle_Koop_Poly3():
    def __init__(self,dt=0.0,x_state=2,u_state=2, l_curr = 0.0, r_curr = 0.0, lv_curr = 0.0, rv_curr = 0.0, ul_curr = 0.0, ur_curr = 0.0):
        """Turtlebot Koopman training class using polynomial basis functions

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            l_curr (float): current left wheel position
            r_curr (float): current right wheel position
            lv_curr(float): current levt wheel velocity
            rv_curr(float): current right wheel velocity
            ur_curr (float): current right wheel command input
            ul_curr (float): current left wheel command input
        """
        
        self.dt = dt
        
        self.state_obs = x_state+8
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        
        self.l_curr = l_curr
        self.r_curr = r_curr
        self.l_old = l_curr
        self.r_old = r_curr
        
        self.lv_curr = lv_curr
        self.rv_curr = rv_curr
        self.lv_old = lv_curr
        self.rv_old = rv_curr
        
        self.ul_curr = ul_curr
        self.ur_curr = ur_curr
        self.ul_old = ul_curr
        self.ur_old = ur_curr
        
        self.divider = 1


    def train_model(self,ltheta,rtheta, lvel, rvel, ul, ur):
        
        """train model parameters using Koopman

        Args:
            ltheta: updated left wheel angle
            rtheta: updated right wheel angle
            lvel: left wheel velocity
            rvel: right wheel velocity
            ul: updated left wheel command
            ur: updated right wheel command
            
        Returns:
            None
        """
        
        self.l_old = self.l_curr
        self.r_old = self.r_curr
        self.ul_old = self.ul_curr
        self.ur_old = self.ur_curr
        self.lv_old = self.lv_curr
        self.rv_old = self.rv_curr
        
        self.l_curr = ltheta
        self.r_curr = rtheta
        self.lv_curr = lvel
        self.rv_curr = rvel
        self.ul_curr = ul
        self.ur_curr = ur
        
        psix_old = self.zee_x(self.l_old, self.r_old,self.lv_old,self.rv_old) #This is an array
        psix_curr = self.zee_x(self.l_curr, self.r_curr, self.lv_curr, self.rv_curr)
        
        psiu_old = self.zee_u(self.ul_old, self.ur_old)
        psiu_curr = self.zee_u(self.ul_curr, self.ur_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        # print(koop_curr.shape)
        # print(koop_old.shape)
        print('polynomial...')
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider               
             
        self.divider+=1


    def zee_x(self,ltheta, rtheta, lvel, rvel):
        """Compute state observation parameters

        Args:
            ltheta: left wheel angle
            rtheta: right wheel angle 
            ul: left wheel command
            ur: right wheel command
        """
        
        return np.array([ltheta, rtheta, lvel, rvel, 1., lvel**2, rvel**2, lvel*rvel, lvel**3, lvel*rvel**2, rvel*lvel**2, rvel**3])
    
    
    def zee_u(self,ul, ur):
        """compute control input observation parameters

        Args:
            ul (float): left wheel command
            ur (float): left wheel command
        """
        
        return np.array([ul, ur])

        

    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
  
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))

        K_reduced = self.K[:4,:]
        print('K reduced : ',  repr(K_reduced))
        
        
        
class Turtle_Koop_Poly4():
    def __init__(self,dt=0.0,x_state=2,u_state=2, l_curr = 0.0, r_curr = 0.0, lv_curr = 0.0, rv_curr = 0.0, ul_curr = 0.0, ur_curr = 0.0):
        """Turtlebot Koopman training class using polynomial basis functions

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            l_curr (float): current left wheel position
            r_curr (float): current right wheel position
            lv_curr(float): current levt wheel velocity
            rv_curr(float): current right wheel velocity
            ur_curr (float): current right wheel command input
            ul_curr (float): current left wheel command input
        """
        
        self.dt = dt
        
        self.state_obs = x_state+13
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        
        self.l_curr = l_curr
        self.r_curr = r_curr
        self.l_old = l_curr
        self.r_old = r_curr
        
        self.lv_curr = lv_curr
        self.rv_curr = rv_curr
        self.lv_old = lv_curr
        self.rv_old = rv_curr
        
        self.ul_curr = ul_curr
        self.ur_curr = ur_curr
        self.ul_old = ul_curr
        self.ur_old = ur_curr
        
        self.divider = 1


    def train_model(self,ltheta,rtheta, lvel, rvel, ul, ur):
        
        """train model parameters using Koopman

        Args:
            ltheta: updated left wheel angle
            rtheta: updated right wheel angle
            lvel: left wheel velocity
            rvel: right wheel velocity
            ul: updated left wheel command
            ur: updated right wheel command
            
        Returns:
            None
        """
        
        self.l_old = self.l_curr
        self.r_old = self.r_curr
        self.ul_old = self.ul_curr
        self.ur_old = self.ur_curr
        self.lv_old = self.lv_curr
        self.rv_old = self.rv_curr
        
        self.l_curr = ltheta
        self.r_curr = rtheta
        self.lv_curr = lvel
        self.rv_curr = rvel
        self.ul_curr = ul
        self.ur_curr = ur
        
        psix_old = self.zee_x(self.l_old, self.r_old,self.lv_old,self.rv_old) #This is an array
        psix_curr = self.zee_x(self.l_curr, self.r_curr, self.lv_curr, self.rv_curr)
        
        psiu_old = self.zee_u(self.ul_old, self.ur_old)
        psiu_curr = self.zee_u(self.ul_curr, self.ur_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        # print(koop_curr.shape)
        # print(koop_old.shape)
        print('polynomial...')
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider               
             
        self.divider+=1


    def zee_x(self,ltheta, rtheta, lvel, rvel):
        """Compute state observation parameters

        Args:
            ltheta: left wheel angle
            rtheta: right wheel angle 
            ul: left wheel command
            ur: right wheel command
        """
        
        return np.array([ltheta, rtheta, lvel, rvel, 1.,rvel**2, lvel*rvel, lvel**2, rvel**3, lvel*rvel**2, (lvel**2)*rvel, lvel**3,rvel**4, lvel*(rvel**3), (lvel**2)*(rvel**2), (lvel**3)*rvel, lvel**4])
    
    
    def zee_u(self,ul, ur):
        """compute control input observation parameters

        Args:
            ul (float): left wheel command
            ur (float): left wheel command
        """
        
        return np.array([ul, ur])

        

    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
  
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))

        K_reduced = self.K[:4,:]
        print('K reduced : ',  repr(K_reduced))
        
        
class Turtle_Koop_Poly4_a():
    def __init__(self,dt=0.0,x_state=2,u_state=2, l_curr = 0.0, r_curr = 0.0, lv_curr = 0.0, rv_curr = 0.0, ul_curr = 0.0, ur_curr = 0.0):
        """Turtlebot Koopman training class using polynomial basis functions

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            l_curr (float): current left wheel position
            r_curr (float): current right wheel position
            lv_curr(float): current levt wheel velocity
            rv_curr(float): current right wheel velocity
            ur_curr (float): current right wheel command input
            ul_curr (float): current left wheel command input
        """
        
        self.dt = dt
        
        self.state_obs = x_state+13
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        
        self.l_curr = l_curr
        self.r_curr = r_curr
        self.l_old = l_curr
        self.r_old = r_curr
        
        self.lv_curr = lv_curr
        self.rv_curr = rv_curr
        self.lv_old = lv_curr
        self.rv_old = rv_curr
        
        self.ul_curr = ul_curr
        self.ur_curr = ur_curr
        self.ul_old = ul_curr
        self.ur_old = ur_curr
        
        self.divider = 1


    def train_model(self,ltheta,rtheta, lvel, rvel, ul, ur):
        
        """train model parameters using Koopman

        Args:
            ltheta: updated left wheel angle
            rtheta: updated right wheel angle
            lvel: left wheel velocity
            rvel: right wheel velocity
            ul: updated left wheel command
            ur: updated right wheel command
            
        Returns:
            None
        """
        
        self.l_old = self.l_curr
        self.r_old = self.r_curr
        self.ul_old = self.ul_curr
        self.ur_old = self.ur_curr
        self.lv_old = self.lv_curr
        self.rv_old = self.rv_curr
        
        self.l_curr = ltheta
        self.r_curr = rtheta
        self.lv_curr = lvel
        self.rv_curr = rvel
        self.ul_curr = ul
        self.ur_curr = ur
        
        psix_old = self.zee_x(self.l_old, self.r_old,self.lv_old,self.rv_old) #This is an array
        psix_curr = self.zee_x(self.l_curr, self.r_curr, self.lv_curr, self.rv_curr)
        
        psiu_old = self.zee_u(self.ul_old, self.ur_old)
        psiu_curr = self.zee_u(self.ul_curr, self.ur_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        # print(koop_curr.shape)
        # print(koop_old.shape)
        print('polynomial...')
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider               
             
        self.divider+=1


    def zee_x(self,ltheta, rtheta, lvel, rvel):
        """Compute state observation parameters

        Args:
            ltheta: left wheel angle
            rtheta: right wheel angle 
            ul: left wheel command
            ur: right wheel command
        """
        
        return np.array([ltheta, rtheta, lvel, rvel, 1.,rvel**2, lvel*rvel, lvel**2, rvel**3, lvel*rvel**2, (lvel**2)*rvel, lvel**3,rvel**4, lvel*(rvel**3), (lvel**2)*(rvel**2), (lvel**3)*rvel, lvel**4])
    
    
    def zee_u(self,ul, ur):
        """compute control input observation parameters

        Args:
            ul (float): left wheel command
            ur (float): left wheel command
        """
        
        return np.array([ul, ur])

        

    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
  
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))

        K_reduced = self.K[:4,:]
        print('K reduced : ',  repr(K_reduced))

class Turtle_Koop_X_Xdot_Poly4():
    def __init__(self,dt=0.0,x_state=6,u_state=2,x_curr = 0.0,xdot_curr=0.0, l_curr = 0.0, r_curr = 0.0, lv_curr = 0.0, rv_curr = 0.0, ul_curr = 0.0, ur_curr = 0.0):
        """Turtlebot Koopman training class using polynomial basis functions

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            x_curr: xposition in meters
            xdot_curr: x velocity in meter/s
            l_curr (float): current left wheel position
            r_curr (float): current right wheel position
            lv_curr(float): current levt wheel velocity
            rv_curr(float): current right wheel velocity
            ur_curr (float): current right wheel command input
            ul_curr (float): current left wheel command input
        """
        
        self.dt = dt
        
        self.state_obs = x_state+13
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        
        self.l_curr = l_curr
        self.r_curr = r_curr
        self.l_old = l_curr
        self.r_old = r_curr
        
        
        self.x_curr = x_curr
        self.x_old = x_curr
        
        self.xdot_curr = xdot_curr
        self.xdot_old = xdot_curr
        
        
        self.lv_curr = lv_curr
        self.rv_curr = rv_curr
        self.lv_old = lv_curr
        self.rv_old = rv_curr
        
        self.ul_curr = ul_curr
        self.ur_curr = ur_curr
        self.ul_old = ul_curr
        self.ur_old = ur_curr
        
        self.divider = 1


    def train_model(self,x, xdot,ltheta,rtheta, lvel, rvel, ul, ur):
        
        """train model parameters using Koopman

        Args:
            x: x linear position
            xdot: xdot linear velocity
            ltheta: updated left wheel angle
            rtheta: updated right wheel angle
            lvel: left wheel velocity
            rvel: right wheel velocity
            ul: updated left wheel command
            ur: updated right wheel command
            
        Returns:
            None
        """
        
        self.l_old = self.l_curr
        self.r_old = self.r_curr
        self.ul_old = self.ul_curr
        self.ur_old = self.ur_curr
        self.lv_old = self.lv_curr
        self.rv_old = self.rv_curr
        
        self.x_old = self.x_curr
        self.xdot_old = self.xdot_curr
        
        self.x_curr = x
        self.xdot_curr = xdot
        
        self.l_curr = ltheta
        self.r_curr = rtheta
        
        self.lv_curr = lvel
        self.rv_curr = rvel
        
        self.ul_curr = ul
        self.ur_curr = ur
        
        psix_old = self.zee_x(self.x_old, self.xdot_old,self.l_old, self.r_old,self.lv_old,self.rv_old) #This is an array
        psix_curr = self.zee_x(self.x_curr,self.xdot_curr, self.l_curr, self.r_curr, self.lv_curr, self.rv_curr)
        
        psiu_old = self.zee_u(self.ul_old, self.ur_old)
        psiu_curr = self.zee_u(self.ul_curr, self.ur_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        # print(koop_curr.shape)
        # print(koop_old.shape)
        print('polynomial...')
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider               
             
        self.divider+=1


    def zee_x(self,x, xdot, ltheta, rtheta, lvel, rvel):
        """Compute state observation parameters

        Args:
            ltheta: left wheel angle
            rtheta: right wheel angle 
            ul: left wheel command
            ur: right wheel command
        """
        
        return np.array([x, xdot, ltheta, rtheta, lvel, rvel, 1.,rvel**2, lvel*rvel, lvel**2, rvel**3, lvel*rvel**2, (lvel**2)*rvel, lvel**3,rvel**4, lvel*(rvel**3), (lvel**2)*(rvel**2), (lvel**3)*rvel, lvel**4])
    
    
    def zee_u(self,ul, ur):
        """compute control input observation parameters

        Args:
            ul (float): left wheel command
            ur (float): left wheel command
        """
        
        return np.array([ul, ur])

        

    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
  
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))

        K_reduced = self.K[:6,:]
        print('K reduced : ',  repr(K_reduced))     


class Turtle_Koop_Vel_X():
    def __init__(self,dt=0.0,x_state=5,u_state=2, x_curr = 0.0, l_curr = 0.0, r_curr = 0.0, lv_curr = 0.0, rv_curr = 0.0, ul_curr = 0.0, ur_curr = 0.0):
        """Turtlebot Koopman training class

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            l_curr (float): current left wheel position
            r_curr (float): current right wheel position
            lv_curr(float): current levt wheel velocity
            rv_curr(float): current right wheel velocity
            ur_curr (float): current right wheel command input
            ul_curr (float): current left wheel command input
        """
        
        self.dt = dt
        
        self.state_obs = x_state+6
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        
        self.x_curr = x_curr
        self.l_curr = l_curr
        self.r_curr = r_curr
        self.x_old = x_curr
        self.l_old = l_curr
        self.r_old = r_curr
        
        self.lv_curr = lv_curr
        self.rv_curr = rv_curr
        # self.lv_old = lv_curr
        # self.rv_old = rv_curr
        
        self.ul_curr = ul_curr
        self.ur_curr = ur_curr
        self.ul_old = ul_curr
        self.ur_old = ur_curr
        
        self.divider = 1


    def train_model(self,x,ltheta,rtheta, lvel, rvel, ul, ur):
        
        """train model parameters using Koopman

        Args:
            x:turtlebot x position
            ltheta: updated left wheel angle
            rtheta: updated right wheel angle
            lvel: left wheel velocity
            rvel: right wheel velocity
            ul: updated left wheel command
            ur: updated right wheel command
            
        Returns:
            None
        """
        print('fourier ... ')
        self.x_old = self.x_curr
        self.l_old = self.l_curr
        self.r_old = self.r_curr
        self.ul_old = self.ul_curr
        self.ur_old = self.ur_curr
        self.lv_old = self.lv_curr
        self.rv_old = self.rv_curr
        
        self.x_curr = x
        self.l_curr = ltheta
        self.r_curr = rtheta
        self.lv_curr = lvel
        self.rv_curr = rvel
        self.ul_curr = ul
        self.ur_curr = ur
        
        psix_old = self.zee_x(self.x_old, self.l_old, self.r_old, self.lv_old, self.rv_old) #This is an array
        psix_curr = self.zee_x(self.x_curr, self.l_curr, self.r_curr, self.lv_curr, self.rv_curr)
        
        psiu_old = self.zee_u(self.ul_old, self.ur_old)
        psiu_curr = self.zee_u(self.ul_curr, self.ur_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider               
             
        self.divider+=1


    def zee_x(self,x, ltheta, rtheta, lvel, rvel):
        """Compute state observation parameters

        Args:
            x: x robot position
            ltheta: left wheel angle
            rtheta: right wheel angle 
            ul: left wheel command
            ur: right wheel command
        """
        
        return np.array([x, ltheta, rtheta, lvel, rvel, 1., np.sin(ltheta),np.cos(ltheta),1., np.sin(rtheta),np.cos(rtheta)])
    
    
    def zee_u(self,ul, ur):
        """compute control input observation parameters

        Args:
            ul (float): left wheel command
            ur (float): left wheel command
        """
        
        return np.array([ul, ur])

        

    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
  
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))

        K_reduced = self.K[:5,:]
        print('K reduced : ',  repr(K_reduced))
        
 
 
 
 
 
class Turtle_Koop_X_Xdot_Poly3():
    def __init__(self,dt=0.0,x_state=6,u_state=2,x_curr = 0.0,xdot_curr=0.0, l_curr = 0.0, r_curr = 0.0, lv_curr = 0.0, rv_curr = 0.0, ul_curr = 0.0, ur_curr = 0.0):
        """Turtlebot Koopman training class using polynomial basis functions

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            x_curr: xposition in meters
            xdot_curr: x velocity in meter/s
            l_curr (float): current left wheel position
            r_curr (float): current right wheel position
            lv_curr(float): current levt wheel velocity
            rv_curr(float): current right wheel velocity
            ur_curr (float): current right wheel command input
            ul_curr (float): current left wheel command input
        """
        
        self.dt = dt
        
        self.state_obs = x_state+7
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        
        self.l_curr = l_curr
        self.r_curr = r_curr
        self.l_old = l_curr
        self.r_old = r_curr
        
        self.x_curr = x_curr
        self.x_old = x_curr
        
        self.xdot_curr = xdot_curr
        self.xdot_old = xdot_curr        
        
        self.lv_curr = lv_curr
        self.rv_curr = rv_curr
        self.lv_old = lv_curr
        self.rv_old = rv_curr
        
        self.ul_curr = ul_curr
        self.ur_curr = ur_curr
        self.ul_old = ul_curr
        self.ur_old = ur_curr
        
        self.divider = 1


    def train_model(self,x, xdot,ltheta,rtheta, lvel, rvel, ul, ur):
        
        """train model parameters using Koopman

        Args:
            x: x linear position
            xdot: xdot linear velocity
            ltheta: updated left wheel angle
            rtheta: updated right wheel angle
            lvel: left wheel velocity
            rvel: right wheel velocity
            ul: updated left wheel command
            ur: updated right wheel command
            
        Returns:
            None
        """
        
        self.l_old = self.l_curr
        self.r_old = self.r_curr
        self.ul_old = self.ul_curr
        self.ur_old = self.ur_curr
        self.lv_old = self.lv_curr
        self.rv_old = self.rv_curr
        
        self.x_old = self.x_curr
        self.xdot_old = self.xdot_curr
        
        self.x_curr = x
        self.xdot_curr = xdot
        
        self.l_curr = ltheta
        self.r_curr = rtheta
        
        self.lv_curr = lvel
        self.rv_curr = rvel
        
        self.ul_curr = ul
        self.ur_curr = ur
        
        psix_old = self.zee_x(self.x_old, self.xdot_old,self.l_old, self.r_old,self.lv_old,self.rv_old) #This is an array
        psix_curr = self.zee_x(self.x_curr,self.xdot_curr, self.l_curr, self.r_curr, self.lv_curr, self.rv_curr)
        
        psiu_old = self.zee_u(self.ul_old, self.ur_old)
        psiu_curr = self.zee_u(self.ul_curr, self.ur_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        # print(koop_curr.shape)
        # print(koop_old.shape)
        print('polynomial...')
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider               
             
        self.divider+=1


    def zee_x(self,x, xdot, ltheta, rtheta, lvel, rvel):
        """Compute state observation parameters

        Args:
            ltheta: left wheel angle
            rtheta: right wheel angle 
            ul: left wheel command
            ur: right wheel command
        """
        
        return np.array([x, xdot, ltheta, rtheta, lvel, rvel, 1., xdot**2, lvel**2, rvel**2, xdot*lvel, xdot*rvel,lvel*rvel])
      
    def zee_u(self,ul, ur):
        """compute control input observation parameters

        Args:
            ul (float): left wheel command
            ur (float): left wheel command
        """
        
        return np.array([ul, ur])

        

    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
  
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))

        K_reduced = self.K[:6,:]
        print('K reduced : ',  repr(K_reduced))     

 
class Turtle_Koop_X_Xdot_Poly3_Theta_Input():
    def __init__(self,dt=0.0,x_state=4,u_state=2,x_curr = 0.0,xdot_curr=0.0,  lv_curr = 0.0, rv_curr = 0.0, ul_curr = 0.0, ur_curr = 0.0):
        """Turtlebot Koopman training class using polynomial basis functions

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            x_curr: xposition in meters
            xdot_curr: x velocity in meter/s
            lv_curr(float): current levt wheel velocity
            rv_curr(float): current right wheel velocity
            ur_curr (float): current right wheel angle input
            ul_curr (float): current left wheel angle input
        """
        
        self.dt = dt
        
        self.state_obs = x_state+7
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        
        self.x_curr = x_curr
        self.x_old = x_curr
        
        self.xdot_curr = xdot_curr
        self.xdot_old = xdot_curr        
        
        self.lv_curr = lv_curr
        self.rv_curr = rv_curr
        self.lv_old = lv_curr
        self.rv_old = rv_curr
        
        self.ul_curr = ul_curr
        self.ur_curr = ur_curr
        self.ul_old = ul_curr
        self.ur_old = ur_curr
        
        self.divider = 1


    def train_model(self,x, xdot, lvel, rvel, ul, ur):
        
        """train model parameters using Koopman

        Args:
            x: x linear position
            xdot: xdot linear velocity
            lvel: left wheel velocity
            rvel: right wheel velocity
            ul: updated left wheel angle
            ur: updated right wheel angle
            
        Returns:
            None
        """

        self.ul_old = self.ul_curr
        self.ur_old = self.ur_curr
        self.lv_old = self.lv_curr
        self.rv_old = self.rv_curr
        
        self.x_old = self.x_curr
        self.xdot_old = self.xdot_curr
        
        self.x_curr = x
        self.xdot_curr = xdot
        
 
        
        self.lv_curr = lvel
        self.rv_curr = rvel
        
        self.ul_curr = ul
        self.ur_curr = ur
        
        psix_old = self.zee_x(self.x_old, self.xdot_old,self.lv_old,self.rv_old) #This is an array
        psix_curr = self.zee_x(self.x_curr,self.xdot_curr,self.lv_curr, self.rv_curr)
        
        psiu_old = self.zee_u(self.ul_old, self.ur_old)
        psiu_curr = self.zee_u(self.ul_curr, self.ur_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        # print(koop_curr.shape)
        # print(koop_old.shape)
        print('polynomial...')
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider               
             
        self.divider+=1


    def zee_x(self,x, xdot, lvel, rvel):
        """Compute state observation parameters

        Args:
            ltheta: left wheel angle
            rtheta: right wheel angle 
            ul: left wheel command
            ur: right wheel command
        """
        
        return np.array([x, xdot, lvel, rvel, 1., xdot**2, lvel**2, rvel**2, xdot*lvel, xdot*rvel,lvel*rvel])
      
    def zee_u(self,ul, ur):
        """compute control input observation parameters

        Args:
            ul (float): left wheel angle
            ur (float): left wheel angle
        """
        print('ul ur ', ul, ur)
        return np.array([ul, ur])

        

    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
  
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))

        K_reduced = self.K[:6,:]
        print('K reduced : ',  repr(K_reduced))
        
        
class Turtle_Koop_X_Xdot_Poly1_Theta_Input_XY():
    def __init__(self,dt=0.0,x_state=6,u_state=2,x_curr = 0.0,y_curr=0.0,xdot_curr=0.0, ydot_curr=0.0,  lv_curr = 0.0, rv_curr = 0.0, ul_curr = 0.0, ur_curr = 0.0):
        """Turtlebot Koopman training class using polynomial basis functions

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            x_curr: xposition in meters
            y_curr: yposition in meters
            xdot_curr: x velocity in meter/s
            ydot_curr: y velocity in meters/s
            lv_curr(float): current levt wheel velocity
            rv_curr(float): current right wheel velocity
            ur_curr (float): current right wheel angle input
            ul_curr (float): current left wheel angle input
        """
        
        self.dt = dt
        
        self.state_obs = x_state+11
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        
        self.x_curr = x_curr
        self.x_old = x_curr
        
        self.y_curr = y_curr
        self.y_old = y_curr
        
        self.xdot_curr = xdot_curr
        self.xdot_old = xdot_curr    
        
        self.ydot_curr = ydot_curr
        self.ydot_old = ydot_curr   
        
        self.lv_curr = lv_curr
        self.rv_curr = rv_curr
        self.lv_old = lv_curr
        self.rv_old = rv_curr
        
        self.ul_curr = ul_curr
        self.ur_curr = ur_curr
        self.ul_old = ul_curr
        self.ur_old = ur_curr
        
        self.divider = 1


    def train_model(self,x, y, xdot, ydot, lvel, rvel, ul, ur):
        
        """train model parameters using Koopman

        Args:
            x: x linear position
            y: y linear position
            xdot: xdot linear velocity
            ydot: ydot linear velocity
            lvel: left wheel velocity
            rvel: right wheel velocity
            ul: updated left wheel angle
            ur: updated right wheel angle
            
        Returns:
            None
        """

        self.ul_old = self.ul_curr
        self.ur_old = self.ur_curr
        self.lv_old = self.lv_curr
        self.rv_old = self.rv_curr
        
        self.x_old = self.x_curr
        self.xdot_old = self.xdot_curr
        
        self.y_old = self.y_curr
        self.ydot_old = self.ydot_curr
        
        self.x_curr = x
        self.xdot_curr = xdot
        
        self.y_curr = y
        self.ydot_curr = ydot        
        
        self.lv_curr = lvel
        self.rv_curr = rvel
        
        self.ul_curr = ul
        self.ur_curr = ur
        
        psix_old = self.zee_x(self.x_old, self.y_old, self.xdot_old, self.ydot_old, self.lv_old,self.rv_old) #This is an array
        psix_curr = self.zee_x(self.x_curr, self.y_curr, self.xdot_curr, self.ydot_curr, self.lv_curr, self.rv_curr)
        
        psiu_old = self.zee_u(self.ul_old, self.ur_old)
        psiu_curr = self.zee_u(self.ul_curr, self.ur_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        # print(koop_curr.shape)
        # print(koop_old.shape)
        print('polynomial...')
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider               
             
        self.divider+=1


    def zee_x(self,x, y, xdot, ydot, lvel, rvel):
        """Compute state observation parameters

        Args:
            x: xpositition in meters
            y: yposition in meters
            xdot: x velocity in meters/s
            ydot: y velocity in meters/s
            ltheta: left wheel angle
            rtheta: right wheel angle 
            ul: left wheel command
            ur: right wheel command
        """
        print('yinternal: ', y)
        return np.array([x, y,xdot,ydot, lvel, rvel, 1., xdot**2,ydot**2, lvel**2, rvel**2, xdot*lvel, xdot*rvel,ydot*lvel,ydot*rvel, lvel*rvel,xdot*ydot])
      
    def zee_u(self,ul, ur):
        """compute control input observation parameters

        Args:
            ul (float): left wheel angle
            ur (float): left wheel angle
        """
        print('ul ur ', ul, ur)
        return np.array([ul, ur])

        

    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
  
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))

        K_reduced = self.K[:6,:]
        print('K reduced : ',  repr(K_reduced))     
        
        
        
        
class Turtle_Koop_X_Xdot_Poly1_Theta_Input_XY_Circle():
    def __init__(self,dt=0.0,x_state=6,u_state=2,x_curr = 0.0,y_curr=0.0,xdot_curr=0.0, ydot_curr=0.0,  lv_curr = 0.0, rv_curr = 0.0, ul_curr = 0.0, ur_curr = 0.0):
        """Turtlebot Koopman training class using polynomial basis functions

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            x_curr: xposition in meters
            y_curr: yposition in meters
            xdot_curr: x velocity in meter/s
            ydot_curr: y velocity in meters/s
            lv_curr(float): current levt wheel velocity
            rv_curr(float): current right wheel velocity
            ur_curr (float): current right wheel angle input
            ul_curr (float): current left wheel angle input
        """
        
        self.dt = dt
        
        self.state_obs = x_state+19
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        
        self.x_curr = x_curr
        self.x_old = x_curr
        
        self.y_curr = y_curr
        self.y_old = y_curr
        
        self.xdot_curr = xdot_curr
        self.xdot_old = xdot_curr    
        
        self.ydot_curr = ydot_curr
        self.ydot_old = ydot_curr   
        
        self.lv_curr = lv_curr
        self.rv_curr = rv_curr
        self.lv_old = lv_curr
        self.rv_old = rv_curr
        
        self.ul_curr = ul_curr
        self.ur_curr = ur_curr
        self.ul_old = ul_curr
        self.ur_old = ur_curr
        
        self.divider = 1


    def train_model(self,x, y, xdot, ydot, lvel, rvel, ul, ur):
        
        """train model parameters using Koopman

        Args:
            x: x linear position
            y: y linear position
            xdot: xdot linear velocity
            ydot: ydot linear velocity
            lvel: left wheel velocity
            rvel: right wheel velocity
            ul: updated left wheel angle
            ur: updated right wheel angle
            
        Returns:
            None
        """

        self.ul_old = self.ul_curr
        self.ur_old = self.ur_curr
        self.lv_old = self.lv_curr
        self.rv_old = self.rv_curr
        
        self.x_old = self.x_curr
        self.xdot_old = self.xdot_curr
        
        self.y_old = self.y_curr
        self.ydot_old = self.ydot_curr
        
        self.x_curr = x
        self.xdot_curr = xdot
        
        self.y_curr = y
        self.ydot_curr = ydot        
        
        self.lv_curr = lvel
        self.rv_curr = rvel
        
        self.ul_curr = ul
        self.ur_curr = ur
        
        psix_old = self.zee_x(self.x_old, self.y_old, self.xdot_old, self.ydot_old, self.lv_old,self.rv_old) #This is an array
        psix_curr = self.zee_x(self.x_curr, self.y_curr, self.xdot_curr, self.ydot_curr, self.lv_curr, self.rv_curr)
        
        psiu_old = self.zee_u(self.ul_old, self.ur_old)
        psiu_curr = self.zee_u(self.ul_curr, self.ur_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        # print(koop_curr.shape)
        # print(koop_old.shape)
        print('polynomial...')
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider               
             
        self.divider+=1


    def zee_x(self,x, y, xdot, ydot, lvel, rvel):
        """Compute state observation parameters

        Args:
            x: xpositition in meters
            y: yposition in meters
            xdot: x velocity in meters/s
            ydot: y velocity in meters/s
            ltheta: left wheel angle
            rtheta: right wheel angle 
            ul: left wheel command
            ur: right wheel command
        """
        print('yinternal: ', y)
        return np.array([x, y,xdot,ydot, lvel, rvel, 1., xdot**2,ydot**2, lvel**2, rvel**2, xdot*lvel, xdot*rvel,ydot*lvel,ydot*rvel, lvel*rvel,xdot*ydot, x**2, y**2, x*y, x*lvel,x*rvel, y*lvel,y*rvel,(x**2)*(y**2)]) #,(x**2)*(y**2)
      
    def zee_u(self,ul, ur):
        """compute control input observation parameters

        Args:
            ul (float): left wheel angle
            ur (float): left wheel angle
        """
        print('ul ur ', ul, ur)
        return np.array([ul, ur])

        

    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
  
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))

        K_reduced = self.K[:6,:]
        print('K reduced : ',  repr(K_reduced))     