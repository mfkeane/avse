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
        self.position = Vector3(x=0.0,y=0.0,z=0.0)
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
	
        self.time = 0.0
	self.dt = 0.02
	self.gpsdt = 0.0
        self.gpstime = 0.0
        self.distance = 0.0

	self.numstates = 4
	#self.heading_time = None
        #self.velocity_time = None
        #self.position_time = None

    def kalman(self):
        #x_k_mat = [x_k, v_k]^T
        #z_k_mat = [z_k]
        #x_(k+1) = x_k + v_k*delta_t + 1/2*a*delta_t^2 = F*x_k + G*a
        #v_(k+1) = v_k + a*delta_t                     = F*x_k + G*a
	#z_k = x_k + epsilon_noise

        delta_t = self.gpsdt

	xk = [[self.position.x, self.position.y], [self.velocity.x, self.velocity.y]]
        ak = [self.acceleration.x, self.acceleration.y]

        F = [[1, delta_t], [0, 1]]
        G = [[0.5*(delta_t)^2], [delta_t]]

        H = [1, 0]

        #x_nextmean = F*x_mean
        #P_nextk = F*Pk*F^T + G*sigma_a^2*G^T

              
	# Measurements, incorporate knowledge of zk into xk
        #y = zk - H*xk
        # R = [sigma_a^2]
        #Sk = H*Pk*H^T + R
        #K = Pk*H^T*Sk^T   #kalman gain
        #x_nextmean_givenz = x_mean + K*y
        #I = eyes() #identity matrix
        #P_nextk_givenz = (I - K*H)*Pk

    def imu_callback(self,imu):
        self.imu = imu
        self.past_acceleration_x.append(self.acceleration.x)
        self.past_acceleration_y.append(self.acceleration.y)
        self.acceleration.x = self.imu.acceleration.x
        self.acceleration.y = self.imu.acceleration.y
        self.past_imu = imu

    def mf_callback(self,mf):
        self.mf = mf
	self.past_heading.append(self.heading)        
	self.heading = math.atan2(self.mf.magnetic_field.y, self.mf.magnetic_field.x)*(180/math.pi) + self.D
        self.past_mf = mf

    def gps_callback(self, gps):
        self.gps = gps
        self.gpsdt = gps.header.stamp.to_sec() - self.gpstime
        self.gpstime = gps.header.stamp.to_sec()

	# Lon/Lat to m
	#RadiusEarth = 6378388.0 # m
	#arc= 2.0*np.pi*(RadiusEarth+gps.altitude)/360.0 # m/degree

        if self.past_gps is not None:
             present = utm.from_latlon(gps.latitude,gps.longitude)
             past = utm.from_latlon(self.past_gps.latitude,self.past_gps.longitude)
             dx2 = present[0]-past[0]
             dy2 = present[1]-past[1]
             self.dx.append(dx2) #(round(dx2,2))
             self.dy.append(dy2) #(round(dy2,2))
	#    self.dx.append(arc * np.cos(gps.latitude*np.pi/180.0) * (gps.longitude-self.past_gps.longitude)) # in m
	#    self.dy.append(arc * (gps.latitude-self.past_gps.latitude)) # in m
        else:
            self.dx.append(0)
	    self.dy.append(0)
	#self.distance = self.distance + np.sqrt(self.dx[-1]**2+self.dy[-1]**2)
       
	self.past_position_x.append(self.position.x)#round(self.position.x,2))
	self.past_position_y.append(self.position.y)#round(self.position.y,2))
	self.position = Vector3(self.past_position_x[-1]+self.dx[-1],self.past_position_y[-1]+self.dy[-1], 0) #round

        self.velocity = Vector3((self.position.x-self.past_position_x[-1])/self.gpsdt, (self.position.y-self.past_position_y[-1])/self.gpsdt, 0)

	self.past_gps = gps
        print("position is: " + str(self.position))
	print("velocity is: " + str(self.velocity))

	x0=self.past_position_x
        x1=self.past_position_y
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
            plt.axis('equal')
            plt.draw()
            plt.pause(0.000000001)
            plt.plot(x0,x1)
            #plt.quiver(x0,x1,np.cos(x2), np.sin(x2), color='#94C600',units='xy', width=0.05, scale=0.5)
#Basic-GPS-Position
            plt.savefig('Basic-GPS-Position.png', dpi=72, transparent=True, bbox_inches='tight')

if __name__ == '__main__':
    rospy.init_node('listener')
    
    server = Server()

    ropsy.Subscriber("/android/imu", Imu, server.imu_callback)
    rospy.Subscriber("/android/magnetometer", MagneticField, server.mf_callback)
    rospy.Subscriber("/android/fix", NavSatFix, server.gps_callback)

    #server.telemetry_plot()
    plt.ion
    plt.show()
    
    rospy.spin()
    

