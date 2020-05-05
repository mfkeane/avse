#!/usr/bin/env python
import rospy
import message_filters
import math
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import NavSatFix, Imu, MagneticField

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

	self.heading_time = None
        self.velocity_time = None
        self.position_time = None

    def kalman(past_position, velocity, heading, model)

    def compute_position(self):
        self.position = kalman(self.past_position, self.velocity, self.heading, self.model)

    def compute_velocity(self):
        # If IMU has produced an update but GPS has not, use IMU
	if (self.imu.header.stamp - self.gps.header.stamp > 0.01) && (self.position is not None):
	    #self.position = self.position + self.velocity * (self.imu.header.stamp-self.position_time)
            self.velocity_time = self.imu.header.stamp

        # Use GPS update when available for accuracy
        else:
            gps_velocity_x = (self.position.x - self.past_position.x)/(self.gps.header.stamp-self.velocity_time)
            gps_velocity_y = (self.position.y - self.past_position.y)/(self.gps.header.stamp-self.velocity_time)
            self.velocity = Vector3(x=gps_velocity_x, y=gps_velocity_y, z=0)
            self.position_time = self.gps.header.stamp
        #vx = etc.
        #vy = etc.
        #self.velocity = Vector3(x=vx, y=vy, z=0)

    def compute_heading(self):
        self.heading = math.atan2(self.mf.magnetic_field.y, self.mf.magnetic_field.x)*(180/math.pi) + self.D

    def compute_vse(self):
        if self.gps is not None and self.imu is not None and self.mf is not None:
            self.compute_velocity()            
            self.compute_heading()
            self.compute_position()

	    self.past_heading = self.heading
            self.past_velocty = self.heading
            self.past_position = self.position



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
        self.compute_vse()
	self.past_imu = imu

    def mf_callback(self, mf):
        self.mf = mf
        #self.mf.magnetic_field.x
        #self.mf.magnetic_field.y
        self.compute_vse()
	self.past_mf = mf


if __name__ == '__main__':
    rospy.init_node('listener')

    server = Server()

    rospy.Subscriber("/android/fix", NavSatFix, server.gps_callback)
    ropsy.Subscriber("/android/imu", Imu, server.imu_callback)
    ropsy.Subscriber("/android/magnetometer", MagneticField, server.mf_callback)



#Notes
# Position can be computed through GPS or IMU, depending on the last recorded update
        # If IMU has produced an update but GPS has not, use IMU
	#if (self.imu.header.stamp - self.gps.header.stamp > 0.01) && (self.position is not None):
	#    self.position = self.position + self.velocity * (self.imu.header.stamp-self.position_time)
        #    self.position_time = self.imu.header.stamp

        # Use GPS update when available for accuracy
        #else:
        #    self.position = Vector3(x=self.gps.latitude, y=self.gps.longitude, z=0)
        #    self.position_time = self.gps.header.stamp

    rospy.spin()

