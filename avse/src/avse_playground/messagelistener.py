#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import NavSatFix, Imu, MagneticField

def callback(gps, imu, mf):
    global gps_lat, gps_long, ox, oy, ax, ay, wx, wy, mf_x, mf_y
    rospy.loginfo(data.latitude)

    gps_lat = gps.latitude
    gps_long = gps.latitude

    ox = imu.orientation.x
    oy = imu.orientation.y
    ax = imu.linear_acceleration.x
    ay = imu.linear_acceleration.y
    wx = imu.angular_velocity.x
    wy = imu.angular_velocity.y

    mf_x = mf.magnetic_field.x
    mf_y = mf.magnetic_field.y


def listener():
    rospy.init_node('listener', anonymous=True)
    gps_sub = message_filters.Subscriber("/android/fix", NavSatFix)
    imu_sub = message_filters.Subscriber("/android/imu", Imu)
    mf_sub = message_filters.Subscriber("/android/magnetometer", MagneticField)
    
    ts = message_filters.ApproximateTimeSynchronizer([gps_sub, imu_sub, mf_sub], 10, 0.05)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
