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

        self.D = 11.824 # Magnetic Declination (Mornington)

	self.heading = None
        self.velocity = None
        self.position = None

    def compute_position(self):
        
        self.position = Vector3(x=self.gps.latitude, y=self.gps.longitude, z=0)

    def compute_velocity(self):
        #vx = etc.
        #vy = etc.
        #self.velocity = Vector3(x=vx, y=vy, z=0)

    def compute_heading(self):
        self.heading = math.atan2(self.mf.magnetic_field.y, self.mf.magnetic_field.x)*(180/math.pi) + self.D

    def compute_vse(self):
        if self.gps is not None and self.imu is not None and self.mf is not None:
            self.compute_position()
            self.compute_velocity()
            self.compute_heading()

    def gps_callback(self, gps):
        self.gps = gps
        #self.gps.latitude
        #self.gps.longitude
        self.compute_vse()

    def gps_callback(self, imu):
        self.imu = imu
        #self.imu.orientation.x
        #self.imu.orientation.y
        #self.imu.linear_acceleration.x
        #self.imu.linear_acceleration.y
        #self.imu.angular_velocity.x
        #self.imu.angular_velocity.y
        self.compute_vse()

    def gps_callback(self, mf):
        self.mf = mf
        #self.mf.magnetic_field.x
        #self.mf.magnetic_field.y
        self.compute_vse()


if __name__ == '__main__':
    rospy.init_node('listener')

    server = Server()

    rospy.Subscriber("/android/fix", NavSatFix, gps_callback)
    ropsy.Subscriber("/android/imu", Imu, imu_callback)
    ropsy.Subscriber("/android/magnetometer", MagneticField, mf_callback)

    rospy.spin()

