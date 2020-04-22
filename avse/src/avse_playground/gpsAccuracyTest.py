#!/usr/bin/env python
import rospy
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

def callback(data):
    rospy.loginfo(data.latitude)
    gps_lat = data.latitude
    gps_long = data.longitude

    str_lat = numpy.float64(gps_lat).astype(str)    
    str_long = numpy.float64(gps_long).astype(str)   

    line = str_lat + "," + str_long + "\n"

    print(line)

    f = open("gpsAccuracyResults.csv","a")

    f.write(line)

    f.close()


def listener():
    
    
    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber("/android/fix", NavSatFix, callback)
    rospy.spin()

if __name__ == '__main__':
    f = open("gpsAccuracyResults.csv","w+")
    f.write("latitude,longitude\n")
    f.close()
    listener()
