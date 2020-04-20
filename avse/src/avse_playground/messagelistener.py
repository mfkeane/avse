#!/usr/bin/env python
import rospy
import message_filters
import math
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Quaternion, Point, PoseWithVelocity
from sensor_msgs.msg import NavSatFix, Imu, MagneticField

class Server:
    def __init__(self):
        self.gps = None
        self.imu = None
        self.mf = None

        self.past_gps = None # use cache
        self.past_imu = None
        self.past_mf = None

        self.D = 11.824 # Magnetic Declination (Mornington)

	self.heading = None
        self.velocity = None
        self.position = None

    def compute_position(self):
        
        self.position = Point(x=self.gps.latitude, y=self.gps.longitude, z=0)

    def compute_velocity(self):
        #use gps and imu data
        #vx = etc.
        #vy = etc.
        #self.velocity = Vector3(x=vx, y=vy, z=0)

    def compute_heading(self):
        #use mag and imu heading, plus kalman fiter to fuse
        self.heading = math.atan2(self.mf.magnetic_field.y, self.mf.magnetic_field.x)*(180/math.pi) + self.D

    def compute_vse(self):
        if self.gps is not None and self.imu is not None and self.mf is not None:
            self.compute_position()
            self.compute_velocity()
            self.compute_heading()

    def callback(self, gps, imu, mf):
        self.gps = gps
        #self.gps.latitude
        #self.gps.longitude

        self.imu = imu
        #self.imu.orientation.x
        #self.imu.orientation.y
        #self.imu.linear_acceleration.x
        #self.imu.linear_acceleration.y
        #self.imu.angular_velocity.x
        #self.imu.angular_velocity.y

        self.mf = mf
        #self.mf.magnetic_field.x
        #self.mf.magnetic_field.y

        self.compute_vse()

    def publisher(self):
        rospy.Publisher("/vse", 

if __name__ == '__main__':
    rospy.init_node('listener')

    server = Server()

    gps_sub = message_filters.Subscriber("/android/fix", NavSatFix)
    imu_sub = message_filters.Subscriber("/android/imu", Imu)
    mf_sub = message_filters.Subscriber("/android/magnetometer", MagneticField)
    
    ts = message_filters.ApproximateTimeSynchronizer([gps_sub, imu_sub, mf_sub], 10, 0.05)
    ts.registerCallback(server.callback)

    rospy.spin()


