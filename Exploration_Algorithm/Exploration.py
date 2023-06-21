#!/usr/bin/env python3

import sys

import math
import rclpy
from rclpy.node import Node
import time
from scipy.spatial.transform import Rotation

import numpy as np
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleOdometry

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray

from squeeze_custom_msgs.msg import ExternalWrenchEstimate

from Yaw_Admitance_Controller import Yaw_Admitance_Controller_Class   # Import Yaw Admitance Controller Class
# from Force_Impedance_Controller import Force_Impedance_Controller_Class   # Import Force Impedance Controller Class
from Position_Admittance_Loop import Position_Admittance_Loop_Class   # Import Position Admittance Loop Class

class OffBoard(Node):

    def __init__(self):

        #self.traj_node = traj_node
        # self.traj_node.get_logger().info("Starting Trajectory Node")  
        super().__init__('odometry_publisher_node')
        print("Starting Trajectory Node")

        self.setpoint_counter = 0
        self.timestamp = 0
        self.count = 0
        ############################
        self.f = np.zeros(3)
        self.f_body = np.zeros(3)
        self.tau = np.zeros(3)
        ############################
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.q = [0.0,0.0,0.0,1.0]
        self.rot_euler = [0.0,0.0,0.0]
        self.yaw_curr = 0.0
        self.yaw_speed = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.yaw_speed_prev = 0.0
        self.x_error = 0.0
        self.y_error = 0.0
        self.z_error = 0.0
        ############################
        self.yaw_admit_counter = 0  # Controller State || 0: Not Running || 1: Running || Acts as reset switch for initial conditions
        self.yaw_admit_obj = Yaw_Admitance_Controller_Class(0.0)
        self.time = 0.0
        self.dt = 0.02              # Wrench callback dt, used in Yaw admitance controller
        ############################
        self.move_direction = 0    # 0: X+ || 1: X- || 2: Y+ || 3: Y-
        self.force_direction = [0,0,0,0]   # 0: X+ || 1: X- || 2: Y+ || 3: Y-
        
        self.contacts = 0
        self.contact_direction_prev = [0,0,0,0]
        self.contact_direction = [0,0,0,0] # [X+, X-, Y+, Y-] Body Frame
        self.curr_state = [1,-0.75,0,0] # [X+, X-, Y+, Y-] Body Frame
        self.prev_state = [0,0,0,0]
        self.fx_buffer = np.zeros(50)
        self.fy_buffer = np.zeros(50)

        self.i = 0
        ############################
        self.rotation_matrix_inv_yaw = [[1,0,0],[0,1,0],[0,0,1]]
        self.yaw_flag = 0
        self.yaw_flag_set = 0
        self.yaw_move_direction = [3,2,0,1]  # Movement should be in the direction perpendicular to direction before yaw [0,3,1,2] -> [3,1,2,0]
        ############################
        self.initial_conditions()
        ############################
        # self.Force_impedance_obj = Force_Impedance_Controller_Class()
        self.Force_impedance_obj = Position_Admittance_Loop_Class()
        ############################
        # Subscribers
        self.odom_sub = self.create_subscription(VehicleOdometry ,"/fmu/vehicle_odometry/out",self.odom_callback,10)
        self.wrench_sub = self.create_subscription(ExternalWrenchEstimate ,'External_Wrench_Estimate',self.wrench_callback,10)
        # self.body_wrench_sub = self.create_subscription(Float32MultiArray,'Wrench/body_wrench',self.wrench_callback,10)

        # Publishers
        self.offboard_control_node=self.create_publisher(OffboardControlMode, "/fmu/offboard_control_mode/in", 10)
        self.trajectory_setpoint_node=self.create_publisher(TrajectorySetpoint, "/fmu/trajectory_setpoint/in", 10)
        self.vehicle_command_node=self.create_publisher(VehicleCommand, "/fmu/vehicle_command/in", 10)

        # Debug Publishers
        self.move_direction_pub = self.create_publisher(Float64, "/Move_Direction", 10)
        self.contacts_pub = self.create_publisher(Float64, "/Contacts", 10)
        self.curr_state_pub = self.create_publisher(Float64MultiArray, "/Current_State", 10)
        self.force_direction_pub = self.create_publisher(Float64MultiArray, "/Force_Direction", 10)
        self.wrench_filt_pub = self.create_publisher(ExternalWrenchEstimate,'/External_Wrench_Estimate_Avg',10)
        self.yaw_generate_pub = self.create_publisher(Float64, "/Yaw_Generate_State", 10)
        self.yaw_rate_filt_pub = self.create_publisher(Float64,"/Yawspeed_filt",10)
        self.yaw_curr_pub = self.create_publisher(Float64,"Yaw_Radians",10)
        ############################

        self.timer_period = 25 # Hz
        self.timer = self.create_timer(1/self.timer_period, self.trajectory_publisher_callback)

        ############################
        while self.count < 10:
            self.arm()
            self.count += 1
            time.sleep(0.1)

        print("Armed")
        print("Offboard")

        self.land_flag = 0
        self.hover_flag = 1
        ############################

    def trajectory_publisher_callback(self):
        
        if(self.rot_euler[0] < 30.0 and self.rot_euler[1] < 30.0):
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(self.x_set,self.y_set,-self.z_set,self.yaw_set)
        else:
            self.land_flag = 1
        if self.land_flag == 1:
            self.land()
            self.disarm()

    def initial_conditions(self):
        """
        Initial Set Points
        """
        self.x_set_prev = 0.0
        self.y_set_prev = -0.6
        self.x_set = -0.55
        self.y_set = -0.0
        self.z_set = 0.55
        self.yaw_set = 0.0
        self.yaw_set_prev = 0.0
        self.move_distance = 0.25
        self.force_threshold = 1.5      
        self.apply_force = 1.25
        self.yaw_count = 0


    def trajectory_state_machine(self): # Called from Wrench Subscriber Callback
        
        self.position_error()
        if(self.z_error > 0.05 and self.hover_flag == 1):
            self.hover_flag = 1
        else:
            self.hover_flag = 0

        if (self.hover_flag == 1):
            # self.hover()
            print("Hovering")
        else:
            self.yaw_speed_check()
            self.collision_direction()
            self.state_update()
            self.trajectory_generate()
            # print("In Trajectory")

    def yaw_speed_check(self): # To check if Drone has rotated and triggers Yaw Generation
        if (self.yaw_flag_set == 0):
            if (self.yaw_speed < -0.4): # If Yaw Rate is less than -0.75 rad/s then yaw anti-clockwise. Rotation will be CCW only since we move RHS wrt force direction
                self.yaw_flag = -1
                self.yaw_flag_set = 1
                # self.move_direction_prev = self.move_direction
                # self.move_direction = self.yaw_move_direction[(self.move_direction)] # Motion after Yaw should be Perpendicular
                self.yaw_st_pt_generate()
                # self.x_set = self.x_set_prev + 0.2
                # self.y_set = self.y_set + self.move_distance
                self.yaw_set = self.yaw_curr # - 0.79 # - 0.79 # Offset by 45 Deg then start yaw generation
                self.yaw_clip = self.yaw_curr_admit - 1.4

                if self.yaw_clip < 0:
                    self.yaw_clip_set = (2*np.pi) - abs(self.yaw_clip)

                self.yaw_admit_counter = 0

            else:
                self.yaw_flag = 0 # No Yaw Control, In Yaw Admittance
                self.yaw_count = 0
                self.yaw_admitance()

    def yaw_st_pt_generate(self):
        if self.move_direction == 0 :
            x = -0.2
            y = - 0.15 # self.move_distance
        if self.move_direction == 1 :
            x = -0.2
            y = 0.15 # self.move_distance
        if self.move_direction == 2 :
            x = 0.15 # self.move_distance
            y = -0.2
        if self.move_direction == 3 :
            x = - 0.15 # self.move_distance
            y = -0.2
        self.st_pt_body_to_world_transform(x,y)
         
    def collision_direction(self): # Checks the direction and number of collisions or 
                                   # Checks when to switch Yaw Generation OFF, when its ON
  
        # self.contacts = 0
        # self.contact_direction = [0, 0, 0, 0] # [+X, -X, +Y, -Y]

        self.fx_buffer[:-1] = self.fx_buffer[1:]
        self.fx_buffer[-1] = self.f_body[0]

        self.fy_buffer[:-1] = self.fy_buffer[1:]
        self.fy_buffer[-1] = self.f_body[1]

        self.fx_avg = np.mean(self.fx_buffer)
        self.fy_avg = np.mean(self.fy_buffer)

        msg = ExternalWrenchEstimate()

        msg.f_x = self.fx_avg
        msg.f_y = self.fy_avg
        msg.timestamp = self.timestamp

        self.wrench_filt_pub.publish(msg)

        # print("Fx_avg: ", self.fx_avg, "Fy_avg: ", self.fy_avg)

        if(self.yaw_flag_set == 1):
            if(abs(self.fx_avg) > 1.6 or abs(self.fy_avg) > 1.6): # If Contact is made turn off Yaw Generation and switch to Yaw Admittance
                self.yaw_flag_set = 0
        else:

            if(self.move_direction == 0 or self.move_direction == 1):
                if(self.fx_avg < -self.force_threshold):
                    self.contact_direction[0] = 1  # Contact Made in Positive X Direction (Body Frame)
                else:
                    self.contact_direction[0] = 0
                if(self.fx_avg > self.force_threshold):
                    self.contact_direction[1] = 1 # Contact Made in Negative X Direction (Body Frame)
                else:
                    self.contact_direction[1] = 0
            if(self.move_direction == 2 or self.move_direction == 3):
                if(self.fy_avg < -self.force_threshold):
                    self.contact_direction[2] = 1  # Contact Made in Positive Y Direction (Body Frame)
                else:
                    self.contact_direction[2] = 0
                if(self.fy_avg > self.force_threshold):
                    self.contact_direction[3] = 1 # Contact Made in Negative Y Direction (Body Frame)
                else:
                    self.contact_direction[3] = 0

            self.contacts = self.contact_direction[0] + self.contact_direction[1] + self.contact_direction[2] + self.contact_direction[3]

    def state_update(self):
            if(self.move_direction == 0 or self.move_direction == 1):
                if(self.contact_direction[0] == 1):
                    self.move_direction = 2
                if(self.contact_direction[1] == 1):
                    self.move_direction = 3

            if(self.move_direction == 2 or self.move_direction == 3):
                if(self.contact_direction[2] == 1):
                    self.move_direction = 1
                if(self.contact_direction[3] == 1):
                    self.move_direction = 0

            if (self.contact_direction[0] == 1 and self.contact_direction[2] == 1): # X+ & Y+
                self.move_direction = 1
            if (self.contact_direction[0] == 1 and self.contact_direction[3] == 1): # X+ & Y-
                self.move_direction = 2
            if (self.contact_direction[1] == 1 and self.contact_direction[2] == 1): # X- & Y+
                self.move_direction = 3
            if (self.contact_direction[1] == 1 and self.contact_direction[3] == 1): # X- & Y-
                self.move_direction = 0

    def force_apply(self):
        # self.force_direction = self.contact_direction - self.contact_direction_prev

        self.force_direction = np.subtract(self.contact_direction, self.contact_direction_prev)
        self.contact_direction_prev[0] = self.contact_direction[0]
        self.contact_direction_prev[1] = self.contact_direction[1]
        self.contact_direction_prev[2] = self.contact_direction[2]
        self.contact_direction_prev[3] = self.contact_direction[3]

        if self.force_direction[0] == 1:
            self.Force_impedance_obj.set_initial_conditions(-self.apply_force, 0.0,'x')
        if self.force_direction[1] == 1:
            self.Force_impedance_obj.set_initial_conditions(+self.apply_force, 0.0,'x')
        if self.force_direction[2] == 1:
            self.Force_impedance_obj.set_initial_conditions(0.0, -self.apply_force,'y')
        if self.force_direction[3] == 1:
            self.Force_impedance_obj.set_initial_conditions(0.0, +self.apply_force,'y') 

        if(self.contacts != 0):
            direction, st_pt = self.Force_impedance_obj.callback(self.x_set,self.y_set,self.fx_avg,self.fy_avg)
            if(direction == 'x'):
                # print("Applying Force in X")
                self.st_pt_body_to_world_transform(st_pt,0.0)
                # self.x_set = st_pt
            elif(direction == 'y'):
                # print("Applying Force in Y")
                self.st_pt_body_to_world_transform(0.0,st_pt)
                # self.y_set = st_pt

    def trajectory_generate(self):

        if self.yaw_flag == 0:
            print("Move Direction: ", self.move_direction,"Contacts:", self.contacts,"Contact Direction: ", self.contact_direction)
            self.force_apply()
            if self.move_direction == 0:
                self.st_pt_body_to_world_transform(self.move_distance,0.0)
            if self.move_direction == 1:
                self.st_pt_body_to_world_transform(-self.move_distance,0.0)
            if self.move_direction == 2:
                self.st_pt_body_to_world_transform(0.0,self.move_distance)
            if self.move_direction == 3:
                self.st_pt_body_to_world_transform(0.0,-self.move_distance)
        elif self.yaw_flag == -1:
            print("Yawing CCW")

            if (self.yaw_count < 310):
                self.yaw_set = self.yaw_set - 0.26*self.dt # 15 deg/sec CCW

                if self.yaw_set < 0:
                    self.yaw_set = (2*np.pi) - abs(self.yaw_set)

            self.yaw_count = self.yaw_count + 1

            # if (self.yaw_count > 300): # 0.26 * 0.02 * 270 = 1.4 radians (80 Degrees)
            #     self.yaw_set = self.yaw_clip_set

        yaw_flag = Float64()
        yaw_flag.data = float(self.yaw_flag)
        self.yaw_generate_pub.publish(yaw_flag)

        direction = Float64()
        direction.data = float(self.move_direction)
        self.move_direction_pub.publish(direction)

        force = Float64MultiArray()
        force.data = [float(self.force_direction[0]),float(self.force_direction[1]),float(self.force_direction[2]),float(self.force_direction[3])]
        self.force_direction_pub.publish(force)

        contact = Float64()
        contact.data = float(self.contacts)
        self.contacts_pub.publish(contact)

    def st_pt_body_to_world_transform(self,x=0.0,y=0.0):
        X = np.array([x,y,1.0])
        rotation_matrix_yaw = self.rot_matrix(round(self.yaw_curr,1))
        # print("Curr Yaw",(round(self.yaw_curr,2)))
        X = np.matmul(rotation_matrix_yaw, X)

        self.x_set = self.x + X[0]
        self.y_set = self.y + X[1]

        self.x_set = round(self.x_set, 4)
        self.y_set = round(self.y_set, 4)

    def rot_matrix(self,theta):
        return np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])

    def position_impedance(self):                   # Give Set Points in X,Y,Z,Yaw - Not NED Frame to match with Accelerometer
      
        self.x_set,self.y_set = self.position_wrench_impedance_obj.callback(self.f[0],self.f[1],self.dt)

        # Add Yaw Admitance If needed

        # Add Trajectory Code Above This Line #
        
        if(self.rot_euler[0] < 30.0 and self.rot_euler[1] < 30.0):
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(self.x_set,self.y_set,-self.z_set,self.yaw_set)
        else:
            self.land_flag = 1

        if self.land_flag == 1:
            self.land()
            self.disarm()

    def yaw_admitance(self):

        if(self.yaw_admit_counter == 0):
            self.yaw_admit_obj.set_xold(self.yaw_curr_admit)
            self.yaw_admit_counter = 1
        else:
            self.yaw_set = self.yaw_admit_obj.callback(self.dt,self.yaw_curr_admit)
            if self.yaw_set < 0:
                self.yaw_set = (2*np.pi) - abs(self.yaw_set)

    def angle_to_2pi(self,angle_in):

        angle = np.deg2rad(angle_in)
        new_angle = np.arctan2(np.sin(angle),np.cos(angle))
        if new_angle < 0:
            new_angle = (2*np.pi) - abs(new_angle)
        
        return round(new_angle,4)

    def odom_callback(self,msg):
        self.timestamp = msg.timestamp
        self.x = msg.x
        self.y = msg.y
        self.z = -msg.z                 # Converted to XYZ Frame, not NED

        self.q[3] = msg.q[0]
        self.q[0] = msg.q[1]
        self.q[1] = msg.q[2]
        self.q[2] = msg.q[3]

        self.rotation_matrix = Rotation.from_quat([msg.q[1],msg.q[2],msg.q[3],msg.q[0]]).as_matrix()
        
        self.rot = Rotation.from_quat(self.q)
        self.rot_euler = self.rot.as_euler('xyz', degrees=True)  # [roll, pitch, yaw]

        self.yaw_curr = self.angle_to_2pi((self.rot_euler[2]))

        self.yaw_curr_admit = self.rot_euler[2] * math.pi / 180.0

        x = Float64()
        x.data = float(self.yaw_curr)
        self.yaw_curr_pub.publish(x)

        self.rotation_matrix_yaw = Rotation.from_euler('z', self.rot_euler[2], degrees=True).as_matrix()
        self.rotation_matrix_inv_yaw = Rotation.from_euler('z', -self.rot_euler[2], degrees=True).as_matrix()

        self.yaw_speed = msg.yawspeed

        self.yaw_speed = self.yaw_speed * 0.1 + self.yaw_speed_prev * 0.9
        self.yaw_speed_prev = self.yaw_speed

        yaw_speed_msg = Float64()
        yaw_speed_msg.data = float(self.yaw_speed)
        self.yaw_rate_filt_pub.publish(yaw_speed_msg)

        # yaw_speed_msg = Float64()
        # yaw_speed_msg.data = self.yaw_speed
        # self.yaw_speed_filt.publish(yaw_speed_msg)

    def wrench_callback(self,msg): # Also calls the Trajectory state machine
        self.f[0] = msg.f_x # msg.data[0] # 
        self.f[1] = msg.f_y # msg.data[1] # 
        self.f[2] = msg.f_z #  msg.data[2] # 

        self.tau[0] = msg.tau_p
        self.tau[1] = msg.tau_q
        self.tau[2] = msg.tau_r

        self.dt = (msg.timestamp - self.time) * 1e-6
        self.dt = round((self.dt),6)
        self.time = msg.timestamp

        # self.dt = 0.01

        self.dt = np.clip(self.dt,0.0,0.05)

        self.f_body = np.matmul(self.rotation_matrix_inv_yaw,self.f)

        self.trajectory_state_machine()

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,1.0)

    def hover(self):
        
        if(self.rot_euler[0] < 30.0 and self.rot_euler[1] < 30.0):
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(self.x_set,self.y_set,-self.z_set,self.yaw_set)
        else:
            self.land_flag = 1

        if self.land_flag == 1:
            self.land()
            self.disarm()
            
    def position_error(self):
        self.x_error = abs(self.x_set - self.x)
        self.y_error = abs(self.y_set - self.y)
        self.z_error = abs(self.z_set - self.z)
        #print(self.z_error)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND,1.0)
        # self.traj_node.get_logger().info("Landing")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,0.0)
        # self.traj_node.get_logger().info("Disarmed")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()

        msg.timestamp = self.timestamp
        # print("Time Stamp",msg.timestamp)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_node.publish(msg)

    def publish_trajectory_setpoint(self,x=0.0,y=0.0,z=0.0, yaw=0.0,vx=math.nan,vy=math.nan,vz=math.nan, yawspeed=math.nan):
        msg = TrajectorySetpoint()
        
        msg.timestamp = self.timestamp
        # print ("Time Stamp: ",msg.timestamp)
        msg.x = x
        msg.y = y
        msg.z = z
        msg.yaw = yaw

        msg.vx = vx
        msg.vy = vy
        msg.vz = vz
        msg.yawspeed = yawspeed
        # print("X: ",msg.x," Y: ",msg.y," Z: ",msg.z," Yaw: ",msg.yaw)

        self.trajectory_setpoint_node.publish(msg)

    def publish_vehicle_command(self,command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.vehicle_command_node.publish(msg)


if __name__ == '__main__':

    rclpy.init()
    # traj_node = Node('Trajectory_Publisher')
    # offboard = OffBoard(traj_node)

    traj_node = OffBoard()
    rclpy.spin(traj_node)

    traj_node.destroy_node()
    rclpy.shutdown()