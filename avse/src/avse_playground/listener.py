#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import NavSatFix, Imu, MagneticField

def gps_callback(data):
    global gps_lat, gps_long
    rospy.loginfo(data.latitude)
    gps_lat = data.latitude
    gps_long = data.latitude


def imu_callback(data):
    global ax, ay, wx, wy #linear acceleration and angular velocity
    rospy.loginfo(data.latitude)
    ax = data.linear_acceleration.x
    ay = data.linear_acceleration.y
    wx = data.angular_velocity.x
    wy = data.angular_velocity.y


def mf_callback(data):
    global mf_x, mf_y

    mf_x = data.magnetic_field.x
    mf_y = data.magnetic_field.y




def listener():
    rospy.init_node('listener', anonymous=True)
    ropsy.Subscriber("/android/fix", NavSatFix, gps_callback)
    rospy.Subscriber("/android/imu", Imu, imu_callback)
    rospy.Subscriber("/android/magnetometer", MagneticField, mf_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
