import numpy as np
from control import lqr

"""
Package source code for lqr control
"""

def test_package():
    print('Testing package...')
    
class Quad_control():
 
    def __init__(self,m_cart=0, m_pole=0  , g=0, l=0):
        """
        LQR controller
        Args: 
            cart mass
            ball mass
            gravitational constant
            rod + ball radius 
        """   
        self.m_cart = m_cart
        self.m_pole = m_pole
        
        A = np.matrix([
            [0,1,0,0],[0,0,g*m_pole/m_cart,0],[0,0,0,1],[0,0,(m_pole*g+m_cart*g)/(m_cart*l),0]    
        ])
        
        B = np.matrix(
            [
                [0],
                [1/m_cart],
                [0],
                [1/(m_cart*l)]
            ]
            
        )
        
        
        Q = np.asmatrix(np.eye(4))
        Q[0,0] = 1 #0.0001 #10
        Q[1,1] = 0.01
        Q[2,2] = 150000000000
        Q[3,3] = 0.1
        
        # R = np.asmatrix([1])
        R = np.asmatrix([0.000001])
        
        self.K,self.S,self.E = lqr(A,B,Q,R)
        
        
        
        
        
        
    def compute_cmd(self,x,xdot,theta,thetadot, r, dt):
        """_summary_

        Args:
            x (float): cart position
            xdot (float): cart velocity
            theta (float): rod angle
            thetadot (float): rod angular velocity
            r (float): wheel radius
            dt (float): timestep

        Returns:
            None
        """
        
        state =np.matrix( [x, xdot,theta,thetadot] ) #positive x needs negative force, others just positive
        print('K: ', self.K)
        print('K :', np.shape(self.K) )
        
        state1 = np.matrix(	[
            [-state[0,0]],
            [state[0,1]],
            [np.arcsin(state[0,3])],
            [-np.asarray(state[0,2])]
            ])
        # F = self.K*state.transpose()
        F = np.dot(self.K,state.transpose())

        # F = self.K*state1

        print('F: ', F)
        
        a = F[0]/(self.m_cart+self.m_pole)
        print('a: ' , a)
        v_cmd = a*dt+xdot
        

        
        # v_cmd = F[0]*dt/((1+1)*self.m_cart) + xdot
        
        w_cmd = v_cmd/r
        
        return v_cmd
        
        
        

        # state=np.asmatrix(state)
        # # print('state: ' , state.shape())
        # error = (state - self.goal)
        # error = np.asmatrix(state)
        # print('error: ' , error.shape())
        # F = self.K*error
        # print('F:  ', F.shape())
    
    
    def return_K(self):
        """
        Args:
            None
        Returns:
            K (double 4x1 matrix): gain matrix
        
        """
        try:
            print(self.K)
        except:
            print('K not defined')
            
class Quad_control_force():
 
    def __init__(self,m_cart=0, m_pole=0  , g=0, l=0, m_ball = 0):
        """
        LQR controller
        Args: 
            cart mass
            ball mass
            gravitational constant
            rod + ball radius 
        """   
        self.m_cart = m_cart
        self.m_pole = m_pole
        self.m_ball = m_ball
        
        A = np.matrix([
            [0,1,0,0],[0,0,g*(m_pole+m_ball)/m_cart,0],[0,0,0,1],[0,0,(m_ball*g + m_pole*g+m_cart*g)/(m_cart*l),0]    
        ])
        
        B = np.matrix(
            [
                [0],
                [1/m_cart],
                [0],
                [1/(m_cart*l)]
            ]
            
        )
        
        
        Q = 0.01*np.asmatrix(np.eye(4))
        # Q[0,0] = 1 #0.0001 #10
        # Q[1,1] = 1
        # Q[2,2] = 1
        # Q[3,3] = 1
        
        # R = np.asmatrix([1])
        R = np.asmatrix([0.01])
        
        self.K,self.S,self.E = lqr(A,B,Q,R)
        print('A_state: ', A)
        print('B_state: ', B)
        
        
        
        
        
        
    def compute_cmd(self,x,xdot,theta,thetadot):
        """_summary_

        Args:
            x (float): cart position
            xdot (float): cart velocity
            theta (float): rod angle
            thetadot (float): rod angular velocity

        Returns:
            Force (x direction)
        """
        
        state = np.matrix( [-x + 10, -xdot,theta,thetadot] ) #positive x needs negative force, others just positive
        f = np.dot(self.K,state.transpose())
        
        
        print('inside control class f: ')
        print(f)
        print(type(f))
        
        print('state: ')
        print(state)
        print(type(state))
        
        # print('x: ')
        # print(x)
        # print(type(x))
        
        # print('xdot: ')
        # print(xdot)
        # print(type(xdot))
        
        # print('theta: ')
        # print(theta)
        # print(type(theta))
        
        # print('thetadot: ')
        # print(thetadot)
        # print(type(thetadot))
        
        
        return f
        
    
    
    def return_K(self):
        """
        Args:
            None
        Returns:
            K (double 4x1 matrix): gain matrix
        
        """
        try:
            print(self.K)
        except:
            print('K not defined')
            
        