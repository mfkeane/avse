#!/usr/bin/env python
import rospy
import message_filters
import math
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import NavSatFix, Imu, MagneticField

import numpy as np
get_ipython().magic(u'matplotlib inline')
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

	self.heading = None
        self.velocity = None
        self.position = None

	self.past_heading = None
        self.past_velocity = None
        self.past_position = None

        self.time = 0.0
	self.dt = 0.02
	self.dtgps = 1
	self.numstates = 4
	#self.heading_time = None
        #self.velocity_time = None
        #self.position_time = None

    def savestates(x, Z, P, K):
    	x0.append(float(x[0]))
    	x1.append(float(x[1]))
    	x2.append(float(x[2]))
    	x3.append(float(x[3]))
    	Zx.append(float(Z[0]))
    	Zy.append(float(Z[1]))    
    	Px.append(float(P[0,0]))
    	Py.append(float(P[1,1]))
    	Pdx.append(float(P[2,2]))
    	Pdy.append(float(P[3,3]))
    	Kx.append(float(K[0,0]))
    	Ky.append(float(K[1,0]))
    	Kdx.append(float(K[2,0]))
    	Kdy.append(float(K[3,0]))

    def kalman(self)

	vs, psis, dts, xs, ys, lats, lons = symbols('v \psi T x y lat lon')

	gs = Matrix([[xs+vs*dts*cos(psis)],
             [ys+vs*dts*sin(psis)],
             [psis],
             [vs]])
	state = Matrix([xs,ys,psis,vs])
	
	gs.jacobian(state)

	P = np.eye(self.numstates)*1000.0

	hs = Matrix([[xs],[ys]])
	JHs=hs.jacobian(state)

	varGPS = 6.0 # Standard Deviation of GPS Measurement
	R = np.diag([varGPS**2.0, varGPS**2.0])

	I = np.eye(self.numstates)

	# Lon/Lat to m
	RadiusEarth = 6378388.0 # m
	arc= 2.0*np.pi*(RadiusEarth+altitude)/360.0 # m/°

	dx = arc * np.cos(latitude*np.pi/180.0) * np.hstack((0.0, np.diff(longitude))) # in m
	dy = arc * np.hstack((0.0, np.diff(latitude))) # in m

	mx = np.cumsum(dx)
	my = np.cumsum(dy)

	ds = np.sqrt(dx**2+dy**2)

	GPS=(ds!=0.0).astype('bool') # GPS Trigger for Kalman Filter

	x = np.matrix([[mx[0], my[0], 0.5*np.pi, 0.0]]).T

	measurements = np.vstack((mx, my))
	# Length of the measurement
	m = measurements.shape[1]

	# Preallocation for Plotting
	x0, x1, x2, x3 = [], [], [], []
	Zx, Zy = [], []
	Px, Py, Pdx, Pdy = [], [], [], []
	Kx, Ky, Kdx, Kdy = [], [], [], []


	for filterstep in range(m):
    
    	    # Time Update (Prediction)
    	    # ========================
    	    # Project the state ahead
    	    # see "Dynamic Matrix"
    	x[0] = x[0] + dt[filterstep]*x[3]*np.cos(x[2])
    	x[1] = x[1] + dt[filterstep]*x[3]*np.sin(x[2])
    	x[2] = (x[2]+ np.pi) % (2.0*np.pi) - np.pi
    	x[3] = x[3]
    
    	# Calculate the Jacobian of the Dynamic Matrix A
    	# see "Calculate the Jacobian of the Dynamic Matrix with respect to the state vector"
    	a13 = -dt[filterstep]*x[3]*np.sin(x[2])
    	a14 = dt[filterstep]*np.cos(x[2])
    	a23 = dt[filterstep]*x[3]*np.cos(x[2])
    	a24 = dt[filterstep]*np.sin(x[2])
    	JA = np.matrix([[1.0, 0.0, a13, a14],
                    [0.0, 1.0, a23, a24],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]])
    
    
    	# Calculate the Process Noise Covariance Matrix
    	sGPS     = 0.5*8.8*dt[filterstep]**2  # assume 8.8m/s2 as maximum acceleration
    	sCourse  = 2.0*dt[filterstep] # assume 0.5rad/s as maximum turn rate
    	sVelocity= 35.0*dt[filterstep] # assume 8.8m/s2 as maximum acceleration

    	Q = np.diag([sGPS**2, sGPS**2, sCourse**2, sVelocity**2])
    
    	# Project the error covariance ahead
    	P = JA*P*JA.T + Q
    
    	# Measurement Update (Correction)
    	# ===============================
    	# Measurement Function
    	hx = np.matrix([[float(x[0])],
                    [float(x[1])]])

    	if GPS[filterstep]: # with 10Hz, every 5th step
        	JH = np.matrix([[1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0]])
    	else: # every other step
        	JH = np.matrix([[0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0]])        
    
    	S = JH*P*JH.T + R
    	K = (P*JH.T) * np.linalg.inv(S)

    	# Update the estimate via
    	Z = measurements[:,filterstep].reshape(JH.shape[0],1)
    	y = Z - (hx)                         # Innovation or Residual
    	x = x + (K*y)

    	# Update the error covariance
    	P = (I - (K*JH))*P

    	# Save states for Plotting
    	savestates(x, Z, P, K)

        plotP()

        plotx() 

        plotxy()

    def plotP():
        fig = plt.figure(figsize=(16,9))
        plt.semilogy(range(m),Px, label='$x$')
        plt.step(range(m),Py, label='$y$')
        plt.step(range(m),Pdx, label='$\psi$')
        plt.step(range(m),Pdy, label='$v$')

        plt.xlabel('Filter Step')
        plt.ylabel('')
        plt.title('Uncertainty (Elements from Matrix $P$)')
        plt.legend(loc='best',prop={'size':22})

    def plotx():
        fig = plt.figure(figsize=(16,16))

        plt.subplot(311)
        plt.step(range(len(measurements[0])),x0-mx[0], label='$x$')
        plt.step(range(len(measurements[0])),x1-my[0], label='$y$')

        plt.title('Extended Kalman Filter State Estimates (State Vector $x$)')
        plt.legend(loc='best',prop={'size':22})
        plt.ylabel('Position (relative to start) [m]')

        plt.subplot(312)
        plt.step(range(len(measurements[0])),x2, label='$\psi$')
        plt.step(range(len(measurements[0])),
             (course/180.0*np.pi+np.pi)%(2.0*np.pi) - np.pi,
             label='$\psi$ (from GPS as reference)')
        plt.ylabel('Course')
        plt.legend(loc='best',prop={'size':16})

        plt.subplot(313)
        plt.step(range(len(measurements[0])),x3, label='$v$')
        plt.step(range(len(measurements[0])),speed/3.6,
             label='$v$ (from GPS as reference)')
        plt.ylabel('Velocity')
        #plt.ylim([0, 30])
        plt.legend(loc='best',prop={'size':16})
        plt.xlabel('Filter Step')

        plt.savefig('Extended-Kalman-Filter-CHCV-State-Estimates.png',
                dpi=72, transparent=True, bbox_inches='tight')
    def plotxy():

        fig = plt.figure(figsize=(16,9))

        # EKF State
        plt.quiver(x0,x1,np.cos(x2), np.sin(x2), color='#94C600',
               units='xy', width=0.05, scale=0.5)
        plt.plot(x0,x1, label='EKF Position', c='k', lw=5)

        # Measurements
        plt.scatter(mx[::5],my[::5], s=50, label='GPS Measurements', marker='+')
        #cbar=plt.colorbar(ticks=np.arange(20))
        #cbar.ax.set_ylabel(u'EPE', rotation=270)
        #cbar.ax.set_xlabel(u'm')

        # Start/Goal
        plt.scatter(x0[0],x1[0], s=60, label='Start', c='g')
        plt.scatter(x0[-1],x1[-1], s=60, label='Goal', c='r')

        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Position')
        plt.legend(loc='best')
        plt.axis('equal')
        #plt.tight_layout()

        plt.savefig('Extended-Kalman-Filter-CHCV-Position.png',
                dpi=72, transparent=True, bbox_inches='tight')

    def compute_heading(self):
        self.heading = math.atan2(self.mf.magnetic_field.y, self.mf.magnetic_field.x)*(180/math.pi) + self.D

    def compute_vse(self):
        if self.gps is not None and self.imu is not None and self.mf is not None:
            self.kalman(self)
            #self.compute_velocity()            
            #self.compute_heading()
            #self.compute_position()

	    #self.past_heading = self.heading
            #self.past_velocty = self.heading
            #self.past_position = self.position



    def gps_callback(self, gps):
        self.gps = gps
        #self.gps.latitude
        #self.gps.longitude
        self.compute_vse()
	self.past_gps = gps

    def imu_callback(self, imu):
        self.imu = imu
        #self.imu.orientation.x
        #self.imu.orientation.y
        #self.imu.linear_acceleration.x
        #self.imu.linear_acceleration.y
        #self.imu.angular_velocity.x
        #self.imu.angular_velocity.y
        #self.compute_vse()
	self.past_imu = imu

    def mf_callback(self, mf):
        self.mf = mf
        #self.mf.magnetic_field.x
        #self.mf.magnetic_field.y
        #self.compute_vse()
	self.past_mf = mf


if __name__ == '__main__':
    rospy.init_node('listener')

    server = Server()

    rospy.Subscriber("/android/fix", NavSatFix, server.gps_callback)
    ropsy.Subscriber("/android/imu", Imu, server.imu_callback)
    ropsy.Subscriber("/android/magnetometer", MagneticField, server.mf_callback)
    rospy.spin()

