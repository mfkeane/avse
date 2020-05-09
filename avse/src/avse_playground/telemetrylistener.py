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

	self.past_heading = []
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


    def telemetry_plot(self):
        x0=self.past_position_x
        x1=self.past_position_y
        x2=self.past_heading

        mx = np.cumsum(self.dx)
	my = np.cumsum(self.dy)

        # Start/Goal
	if len(x0)==1:
            fig = plt.figure(figsize=(16,9))
            # EKF State
       	    #plt.quiver(x0,x1,np.cos(x2), np.sin(x2), color='#94C600',units='xy', width=0.05, scale=0.5)
            #plt.plot(x0,x1, label='Position', c='k', lw=5)

            # Measurements
            #plt.scatter(mx[::5],my[::5], s=50, label='GPS Measurements', marker='+')
            #cbar=plt.colorbar(ticks=np.arange(20))
            #cbar.ax.set_ylabel(u'EPE', rotation=270)
            #cbar.ax.set_xlabel(u'm')	
            
            #plt.scatter(x0[0],x1[0], s=60, label='Start', c='g')
            #plt.scatter(x0[-1],x1[-1], s=60, label='Goal', c='r')

            plt.xlabel('X [m]')
            plt.ylabel('Y [m]')
            plt.title('Position')
            #plt.legend(loc='best')
            #plt.axis('equal')
            #plt.tight_layout()

        
	if len(x0)>0:
            plt.plot(x0[-1],x1[-1],'*')
            plt.axis('equal')
            plt.draw()
            plt.pause(0.000000001)
	    plt.savefig('Basic-GPS-Position.png', dpi=72, transparent=True, bbox_inches='tight')

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

    rospy.Subscriber("/android/fix", NavSatFix, server.gps_callback)
    #ropsy.Subscriber("/android/imu", Imu, server.imu_callback)
    rospy.Subscriber("/android/magnetometer", MagneticField, server.mf_callback)

    #server.telemetry_plot()
    plt.ion
    plt.show()
    rospy.spin()

