import numpy as np


class Position_Admittance_Loop_Class:
    def __init__(self) -> None:
        # self.m = 1.0
        # self.k_x = 30.0 # 24.5
        # self.k_y = 24.5
        # self.b_x = 1.0 # 24.5
        # self.b_y = 24.5

        self.m = 1.0
        self.k_x = 24.5
        self.k_y = 24.5
        self.b_x = 24.5
        self.b_y = 24.5

        # self.fext_gain = 2.0
        self.fext_gain = 1.0

    def set_initial_conditions(self,fx_des,fy_des,force_direction):
        """
        Enter Desired Force Here
        """
        self.fx_des = fx_des
        self.fy_des = fy_des
        """
        Force Direction to be set here
        """
        self.force_direction = force_direction
        """
        Resetting the values
        """
        self.x_set_dot_new = 0.0
        self.y_set_dot_new = 0.0

    # def callback(self, x_sp_curr, y_sp_curr, fx_curr, fy_curr):
        
    #     dt = 0.001
    #     n = 100
    #     x_set_new = x_sp_curr
    #     x_set_dot_new = 0.0
    #     y_set_new = y_sp_curr
    #     y_set_dot_new = 0.0
        
    #     if self.force_direction == 'x':
    #         # x_set_dot_dot_new = ((-self.fx_des + fx_curr) + (self.k_x* x_sp_curr) - (self.b_x * self.x_set_dot_new) - (self.k_x * x_set_new))
    #         x_set_dot_dot_new = ((-self.fx_des + fx_curr)*self.fext_gain + (self.k_x* x_sp_curr) - (self.b_x * x_set_dot_new) - (self.k_x * x_set_new))
    #         x_set_dot_new = x_set_dot_new + (x_set_dot_dot_new * dt)
    #         x_set_new = x_set_new + (x_set_dot_new * dt)

    #         return 'x', x_set_new

    #     if self.force_direction == 'y':
    #         y_set_dot_dot_new = ((-self.fy_des + fy_curr) + (self.k_y* y_sp_curr) - (self.b_y * self.y_set_dot_new) - (self.k_y * y_set_new))
    #         self.y_set_dot_new = self.y_set_dot_new + (y_set_dot_dot_new * dt)
    #         y_set_new = y_set_new + (self.y_set_dot_new * dt)
            
    #         return 'y', y_set_new
    
    def callback(self, x_sp_curr, y_sp_curr, fx_curr, fy_curr):
        
        dt = 0.001
        n = 100
        x_set_new = x_sp_curr
        x_set_dot_new = 0.0
        y_set_new = y_sp_curr
        y_set_dot_new = 0.0
        
        if self.force_direction == 'x':
            for i in range(n):
                #if(x_sp_curr >= 0):
                x_set_dot_dot_new = ((-self.fx_des + fx_curr)*self.fext_gain + (self.k_x* x_sp_curr) - (self.b_x * x_set_dot_new) - (self.k_x * x_set_new))
                #else :
                # x_set_dot_dot_new = ((self.fx_des - fx_curr) + (self.k_x* x_sp_curr) - (self.b_x * x_set_dot_new) - (self.k_x * x_set_new))

                x_set_dot_new = x_set_dot_new + (x_set_dot_dot_new * dt)
                x_set_new = x_set_new + (x_set_dot_new * dt)

            return 'x', x_set_new

        if self.force_direction == 'y':
            for i in range(n):
                #if(y_sp_curr >=0):
                y_set_dot_dot_new = ((-self.fy_des + fy_curr) + (self.k_y* y_sp_curr) - (self.b_y * y_set_dot_new) - (self.k_y * y_set_new))
                #else :
                #    y_set_dot_dot_new = ((self.fy_des - fy_curr) + (self.k_y* y_sp_curr) - (self.b_y * y_set_dot_new) - (self.k_y * y_set_new))

                y_set_dot_new = y_set_dot_new + (y_set_dot_dot_new * dt)
                y_set_new = y_set_new + (y_set_dot_new * dt)
            
            return 'y', y_set_new