#!/usr/bin/env python
import rospy
import message_filters
import math
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import NavSatFix, Imu, MagneticField


import utm

import numpy as np
#%matplotlib inline
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
from scipy.stats import norm
from sympy import Symbol, symbols, Matrix, sin, cos
from sympy import init_printing
from sympy.utilities.codegen import codegen
init_printing(use_latex=True)



#-----------------------------------------------------------

class Server:
    def __init__(self):
        self.gps = None
        self.imu = None
        self.mf = None

        self.past_gps = None
        self.past_imu = None
        self.past_mf = None

        self.D = 11.824 # Magnetic Declination (Mornington)

	self.heading = 0.0
        self.velocity = Vector3(x=0.0,y=0.0,z=0.0)
        self.k_velocity = Vector3(x=0.0,y=0.0,z=0.0)
        self.position = Vector3(x=0.0,y=0.0,z=0.0)
        self.k_position = Vector3(x=0.0,y=0.0,z=0.0)
        self.acceleration = Vector3(x=0.0,y=0.0,z=0.0)

	self.past_heading = []
        self.past_acceleration_x = []
        self.past_acceleration_y = []
        self.past_velocity_x = []
        self.past_velocity_y = []
        self.past_position_x = []
        self.past_position_y = []
	
	self.dx = []
        self.dy = []
	
	self.x_k = np.array([[0],[0],[0],[0]])
        #self.Pk = np.array([[0,0],[0,0]])
	#self.x_next = np.array([[0,0],[0,0]])
        #self.P_next = np.array([[0,0],[0,0]])

        self.kf_past_velocity_x = []
        self.kf_past_velocity_y = []
        self.kf_past_position_x = []
        self.kf_past_position_y = []
        self.P = np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])

        self.time = 0.0
	self.dt = 0.02
	self.gpsdt = 0.0
        self.gpstime = 0.0
        self.distance = 0.0

	self.pitch = 0.0;
	self.roll = 0.0;
        self.yaw =0.0;

	self.numstates = 4
	#self.heading_time = None
        #self.velocity_time = None
        #self.position_time = None

    def kalman(self):
        # Update time rate
        delta_t = self.gpsdt
	#print(delta_t)

	
	# Process
	xk_1 = self.x_k
        ak_1 = np.array([[self.acceleration.x], [self.acceleration.y]])
        A = np.array([[1,0,delta_t,0],[0,1,0,delta_t],[0,0,1,0],[0,0,0,1]])
        B = np.array([[0.5*(delta_t*delta_t),0],[0,0.5*(delta_t*delta_t)],[delta_t,0],[0,delta_t]])

        # Measurements
        zk = np.array([[self.position.x], [self.position.y]])
        H = np.array([[1,0,0,0],[0,1,0,0]])

        # Error Matrices
        # Disturbance Covariances (model)
	Q = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        # Noise Covariances (Sensors)
        R = np.array([[1,0],[0,1]])
                    #[[self.gps.position_covariance[0],0],[0,self.gps.position_covariance[4]]])

	# Prediction Equations
        X_k = A.dot(xk_1) + B.dot(ak_1)
        p = A.dot(self.P).dot(A.transpose()) + Q

	# Update Equations
  	K = p.dot(H.transpose()).dot(np.linalg.inv(H.dot(p).dot(H.transpose())+R))
        self.x_k = X_k + K.dot(zk - H.dot(X_k))
        self.P = (np.identity(4)-K.dot(H)).dot(p)

	self.kf_past_position_x.append(self.x_k[0])
        self.kf_past_position_y.append(self.x_k[1])
        self.kf_past_velocity_x.append(self.x_k[2])
        self.kf_past_velocity_y.append(self.x_k[3])
        self.kf_position = (self.x_k[0],self.x_k[1],0.0)

	
    def imu_callback(self,imu):
        self.imu = imu
        self.past_acceleration_x.append(self.acceleration.x)
        self.past_acceleration_y.append(self.acceleration.y)
        self.past_heading.append(self.heading)

        # Calculate Heading (Yaw orientation)
        w = self.imu.orientation.w
        x = self.imu.orientation.x
        y = self.imu.orientation.y
        z = self.imu.orientation.z      

        t0 = +2.0 * (w * x + y * z)
    	t1 = +1.0 - 2.0 * (x * x + y * y)
        self.roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        self.pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        self.yaw = math.atan2(t3, t4)
    
        self.heading = self.yaw

	# Zero acceleration due to gravity
        self.acceleration.x = self.imu.linear_acceleration.x - 9.80665*sin(self.pitch) -1.4#-1.4?
        self.acceleration.y = self.imu.linear_acceleration.y - 9.80665*sin(self.roll)
	print("Acceleration x is "+ str(self.acceleration.x))
	print("Acceleration y is "+ str(self.acceleration.y))
	# sanity check
        #print(math.sqrt(self.imu.linear_acceleration.x*self.imu.linear_acceleration.x + self.imu.linear_acceleration.z*self.imu.linear_acceleration.z))
        #print(math.sqrt(self.imu.linear_acceleration.y*self.imu.linear_acceleration.y + self.imu.linear_acceleration.z*self.imu.linear_acceleration.z))

	self.past_imu = imu

    def mf_callback(self,mf):
        self.mf = mf
	#self.past_heading.append(self.heading)
	#if self.imu is not None:  

	#self.heading = math.atan2(self.mf.magnetic_field.y, self.mf.magnetic_field.x)*(180/math.pi) + self.D
        #print(self.heading)
        #print(imu_heading)
        self.past_mf = mf

    def gps_callback(self, gps):
        self.gps = gps
        self.gpsdt = gps.header.stamp.to_sec() - self.gpstime
	if self.gpsdt>5:
	    self.gpsdt=1.0
        self.gpstime = gps.header.stamp.to_sec()

        if self.past_gps is not None:
             present = utm.from_latlon(gps.latitude,gps.longitude)
             past = utm.from_latlon(self.past_gps.latitude,self.past_gps.longitude)
             dx2 = present[0]-past[0]
             dy2 = present[1]-past[1]
             self.dx.append(dx2)
             self.dy.append(dy2)

        else:
            self.dx.append(0)
	    self.dy.append(0)
       
	self.past_position_x.append(self.position.x)
	self.past_position_y.append(self.position.y)
	self.position = Vector3(self.past_position_x[-1]+self.dx[-1],self.past_position_y[-1]+self.dy[-1], 0)

        self.velocity = Vector3((self.position.x-self.past_position_x[-1])/self.gpsdt, (self.position.y-self.past_position_y[-1])/self.gpsdt, 0)

	self.kalman()

        

	self.past_gps = gps
        #print("position is: " + str(self.position))
	#print("revised position is: " + str(self.kf_position))

	x0=self.past_position_x
        x1=self.past_position_y
        #print(x0,x1)
        x2=self.past_heading

        mx = np.cumsum(self.dx)
	my = np.cumsum(self.dy)

        # Start/Goal
	if len(x0)==1:
            fig = plt.figure(figsize=(16,9))
        
            plt.xlabel('X [m]')
            plt.ylabel('Y [m]')
            plt.title('Position')
           
	if len(x0)>0:
            plt.plot(x0[-1],x1[-1],'*')
            plt.plot(self.kf_past_position_x, self.kf_past_position_y)
            plt.axis('equal')
            plt.draw()
            plt.pause(0.000000001)
            #plt.plot(x0,x1)
            #plt.plot(self.kf_past_position_x, self.kf_past_position_y)
  
            #Basic-GPS-Position
            plt.savefig('KF1-GPS-Position.png', dpi=72, transparent=True, bbox_inches='tight')

if __name__ == '__main__':
    rospy.init_node('listener')
    
    server = Server()

    rospy.Subscriber("/android/imu", Imu, server.imu_callback)
    rospy.Subscriber("/android/magnetometer", MagneticField, server.mf_callback)
    rospy.Subscriber("/android/fix", NavSatFix, server.gps_callback)

    #server.telemetry_plot()
    plt.ion
    plt.show()
    
    rospy.spin()
    

