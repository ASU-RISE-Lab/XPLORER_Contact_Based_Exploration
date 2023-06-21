import numpy as np

class Yaw_Admitance_Controller_Class:
    def __init__(self,initial_yaw):
        self.x_old = np.matrix([[initial_yaw], [0]])
        self.x_curr = np.matrix('0 ; 0')
        self.A = np.matrix( '0 1 ; -100 -20' )
        self.B = np.matrix('0 ; 100')
        self.C = np.matrix('1 0')

    def set_xold(self,initial_yaw):
        self.x_old = np.matrix([[initial_yaw], [0]])

    def callback(self,dt,yaw_curr):

        self.u_old = yaw_curr
        self.A_d = self.A * dt + np.eye(2)                  # A_d = I + A * dt
        self.B_d = self.B * dt                              # B_d = B * dt

        self.x_curr = self.A_d*self.x_old + self.B_d*self.u_old  # x(k+1) = A_d*x(k) + B_d*u(k)

        self.x_old = self.x_curr
        self.yaw_set = round(self.x_curr.item(0,0),4)

        # print(self.yaw_set)

        return self.yaw_set

