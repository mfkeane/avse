#!/usr/bin/env python
import rospy
import message_filters
import math
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Vector3, Quaternion, PoseWithCovariance, TwistWithCovariance, Pose, Point, Twist, Vector3Stamped
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from nav_msgs.msg import Odometry

import utm

import numpy as np
# %matplotlib inline
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
from scipy.stats import norm
from sympy import Symbol, symbols, Matrix, sin, cos
from sympy import init_printing
from sympy.utilities.codegen import codegen
init_printing(use_latex=True)

#from filterpy.kalman import KalmanFilter

# -----------------------------------------------------------


class Server:
    def __init__(self):
        self.gps = None
        self.imu = None
        self.mf = None
        self.gps_velocity = None

        self.past_gps = None
        self.past_imu = None
        self.past_mf = None

        self.calibrated = False

        self.D = 11.824  # Magnetic Declination (Mornington)

        self.firstimu = 1
        self.headingimu = 0.0
        self.firstheadingimu = 0.0
        self.angular_velocity = 0.0

        self.firstodom = 1
        self.headingodom = 0.0
        self.firstheadingodom = 0.0
        self.angular_velocity_gps = 0.0

        self.velocity = Vector3(x=0.0, y=0.0, z=0.0)
        self.k_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        self.position = Vector3(x=0.0, y=0.0, z=0.0)
        self.k_position = Vector3(x=0.0, y=0.0, z=0.0)
        self.acceleration = Vector3(x=0.0, y=0.0, z=0.0)

        self.firstgps = 0
        self.firstposition = [0, 0]
        self.past_headingimu = []
        self.past_headinggps = []
        self.past_acceleration_x = []
        self.past_acceleration_y = []
        self.past_acceleration_z = []
        self.past_velocity_x = []
        self.past_velocity_y = []
        self.past_velocity_z = []
        self.past_position_x = []
        self.past_position_y = []
        self.past_position_z = []

        self.dx = 0
        self.dy = 0

        self.x_k = np.array([[0], [0], [0], [0]])

        self.acc_k = np.array([[0], [0], [0], [0]])

        #self.Pk = np.array([[0,0],[0,0]])
        #self.x_next = np.array([[0,0],[0,0]])
        #self.P_next = np.array([[0,0],[0,0]])

        self.kf_past_velocity_x = []
        self.kf_past_velocity_y = []
        self.kf_past_position_x = []
        self.kf_past_position_y = []
        self.P = np.array([[0.01, 0, 0, 0], [0, 0.64, 0, 0],
                           [0, 0, 0.01, 0], [0, 0, 0, 0.01]])

        self.accP = np.array([[0.01, 0, 0, 0], [0, 0.1, 0, 0], [
                             0, 0, 0.01, 0], [0, 0, 0, 0.1]])

        self.start_time = 0.0
        self.time = 0.0
        self.dt = 0.02
        self.gpsdt = 0.0
        self.gpstime = 0.0
        self.imudt = 0.0
        self.imutime = 0.0
        self.distance = 0.0

        self.pitchimu = 0.0
        self.rollimu = 0.0
        self.yawimu = 0.0
        self.pitchgps = 0.0
        self.rollgps = 0.0
        self.yawgps = 0.0

        self.numstates = 4
        #self.heading_time = None
        #self.velocity_time = None
        #self.position_time = None

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
             np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
             np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
             np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
             np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def pi_2_pi(self, angle):
        return (angle + np.pi) % (2*np.pi)-np.pi

    def kalman(self):
        # Update time rate
        delta_t = self.gpsdt
        current_time = rospy.Time.now()

        if self.x_k[3] == 0.:
            self.start_time = current_time

        # POSITION AND VELOCITY
        # Process
        xk_1 = self.x_k  # [pos_x, pos_y, vel_x, vel_y]
        ak_1 = np.array([[self.acceleration.x], [self.acceleration.y]])
        A = np.array([[1, 0, delta_t, 0], [0, 1, 0, delta_t],
                      [0, 0, 1, 0], [0, 0, 0, 1]])
        B = np.array([[0.5*(delta_t*delta_t), 0],
                      [0, 0.5*(delta_t*delta_t)], [delta_t, 0], [0, delta_t]])

        # Measurements
        zk = np.array([[self.position.x], [self.position.y],
                       [self.velocity.x], [self.velocity.y]])
        H = np.array([[1, 0, delta_t, 0], [0, 1, 0, delta_t],
                      [0, 0, 1, 0], [0, 0, 0, 1]])

        # Error Matrices
        # Disturbance Covariances (model)
        Q = np.array([[0.05, 0.025, 0, 0], [0.025, 0.025, 0, 0],
                      [0, 0, 0.05, 0.025], [0, 0, 0.025, 0.025]])
        # Noise Covariances (Sensors)
        R = np.array([[20, 0, 0, 0], [0, 20, 0, 0],
                      [0, 0, 2, 0], [0, 0, 0, 2]])
        # [[self.gps.pose.covariance[0],0],[0,self.gps.pose.covariance[4]]])

        # Prediction Equations
        # State Prediction X = Ax + Bu
        X_k = A.dot(xk_1) + B.dot(ak_1)
        # Covariance Prediction
        p = A.dot(self.P).dot(A.transpose()) + Q

        # Update Equations
        # Kalman Gain
        K = p.dot(H.transpose()).dot(
            np.linalg.inv(H.dot(p).dot(H.transpose())+R))
        # State Update x_(n,n-1) + K_n*(z_n - x_(n,n-1))
        self.x_k = X_k + K.dot(zk - H.dot(X_k))
        # Covariance Update (1-K_n)p_(n,n-1)
        self.P = (np.identity(4)-K.dot(H)).dot(p)
        # print(self.x_k)
        self.kf_past_position_x.append(self.x_k[0]) # ! does this become an infinitely large list?
        self.kf_past_position_y.append(self.x_k[1])
        self.kf_past_velocity_x.append(self.x_k[2])
        self.kf_past_velocity_y.append(self.x_k[3])
        self.kf_position = (self.x_k[0], self.x_k[1], 0.0)
        #print("x_k: "+str(self.x_k))

        # ORIENTATION
        # need to do kalm filter on gyro for this
        # Process
        acck_1 = self.acc_k  # [theta; bias; thetagps;biasgps]
        qk_1 = np.array([[self.angular_velocity], [self.angular_velocity_gps]])
        Aacc = np.array([[1, -self.imudt, 0, 0], [0, 1, 0, 0],
                         [0, 0, 1, -delta_t], [0, 0, 0, 1]])
        Bacc = np.array([[self.imudt, 0], [0, 0], [0, delta_t], [0, 0]])

        # Measurements
        zk = np.array([[self.pi_2_pi(self.yawimu)],
                       [self.pi_2_pi(self.yawgps)]])
        H = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])

        # Error Matrices
        # Disturbance Covariances (model)
        Q = np.array([[np.power(self.imudt, 2)*1, 0, 0, 0], [0, 1,
                                                             0, 0], [0, 0, np.power(delta_t, 2)*1, 0], [0, 0, 0, 1]])
        # Noise Covariances (Sensors)
        R = np.array([[0.01, 0], [0, 1]])  # np.array([1])
        # [[self.gps.pose.covariance[0],0],[0,self.gps.pose.covariance[4]]])

        # Prediction Equations
        # State Prediction X = Ax + Bu
        X_k = self.pi_2_pi(Aacc.dot(acck_1) + Bacc.dot(qk_1))
        # Covariance Prediction
        p = Aacc.dot(self.accP).dot(Aacc.transpose()) + Q

        # Update Equations
        # Kalman Gain
        K = p.dot(H.transpose()).dot(
            np.linalg.pinv(H.dot(p).dot(H.transpose())+R))
        # State Update x_(n,n-1) + K_n*(z_n - x_(n,n-1))
        self.acc_k = X_k + K.dot(self.pi_2_pi(zk - H.dot(X_k)))
        self.acc_k[0] = self.pi_2_pi(self.acc_k[0])
        self.acc_k[2] = self.pi_2_pi(self.acc_k[2])
        # print(self.acc_k)
        # Covariance Update (1-K_n)p_(n,n-1)
        self.accP = (np.identity(4)-K.dot(H)).dot(p)

        # 30 seconds of calibration before vehicle starts
        calibration_bool_pub = rospy.Publisher(
            '/mur/CalibratingGPS', Bool, queue_size=10)
        calibration = Bool()
        if (current_time.to_sec() - self.start_time.to_sec()) > 10.0:
            calibration.data = True
            self.calibrated = True
        else:
            calibration.data = False
        calibration_bool_pub.publish(calibration)
        #print((current_time.to_sec() - self.start_time.to_sec()))

        #odom_pub = rospy.Publisher('odometry_publisher', Odometry)
        odom_pub = rospy.Publisher('/mur/Odom', Odometry, queue_size=10)
        #odom_broadcaster = tf.TransformBroadcaster()

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        # print(np.arctan2(self.x_k[3].astype(float),self.x_k[2].astype(float)))
        if self.x_k[3] == 0.:
            odom.pose.pose = Pose(
                Point(self.x_k[0], self.x_k[1], 0.), self.quaternion_from_euler(0., 0., 0.))
        else:
            odom.pose.pose = Pose(Point(self.x_k[0], self.x_k[1], 0.), self.quaternion_from_euler(
                0, 0, self.pi_2_pi(self.acc_k[0])))  # +self.acc_k[0]-self.acc_k[2])))
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(
            Vector3(self.x_k[2], self.x_k[3], 0.), Vector3(0., 0., 0.))
        # print(self.acc_k[0])
        odom_pub.publish(odom)

        # Publish Raw IMU for Path Following Control

        if self.imu is not None:
            imu_pub = rospy.Publisher('/mur/Imu/filtered', Imu, queue_size=10)

            imu_msg = self.imu
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = "imu"
            if self.x_k[3] == 0.:
                imu_msg.orientation = self.quaternion_from_euler(0., 0., 0.)
            else:
                imu_msg.orientation = self.quaternion_from_euler(
                    0, 0, self.pi_2_pi(self.acc_k[0]))
                imu_msg.linear_acceleration = Vector3(
                    self.acceleration.x, self.acceleration.y, 0.0)
            imu_pub.publish(imu_msg)
            # r.sleep()

    def imu_callback(self, imu):
        current_time = rospy.Time.now()
        # Initialise
        self.imu = imu
        self.past_acceleration_x.append(self.acceleration.x)
        self.past_acceleration_y.append(self.acceleration.y)
        self.past_headingimu.append(self.headingimu)

        self.imudt = imu.header.stamp.to_sec() - self.imutime
        if self.imudt > 1:  # Double check this!
            self.gpsdt = 0.02
        self.imutime = imu.header.stamp.to_sec()

        # Calculate Heading (Yaw orientation)
        #euler = euler_from_quaternion(self.imu.orientation)
        #rollimu = euler[0]
        #pitchimu = euler[1]
        #yawimu = euler[2]
        w = self.imu.orientation.w
        x = self.imu.orientation.x
        y = self.imu.orientation.y
        z = self.imu.orientation.z

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        self.rollimu = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        self.pitchimu = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        self.yawimu = math.atan2(t3, t4)
        # if self.firstimu == 1:
        #    self.firstheadingimu = self.yawimu
        #self.headingimu = self.yawimu - self.firstheadingimu
        self.yawimu = self.pi_2_pi(self.yawimu)
        #self.headingimu = self.pi_2_pi(self.headingimu)

        # Zero acceleration due to gravity
        self.acceleration.x = self.imu.linear_acceleration.x - \
            9.80665*sin(self.pitchimu)
        self.acceleration.y = self.imu.linear_acceleration.y - \
            9.80665*sin(self.rollimu)
        #print("Acceleration x is "+ str(self.acceleration.x))
        #print("Acceleration y is "+ str(self.acceleration.y))
        # sanity check
        #print(math.sqrt(self.imu.linear_acceleration.x*self.imu.linear_acceleration.x + self.imu.linear_acceleration.z*self.imu.linear_acceleration.z))
        #print(math.sqrt(self.imu.linear_acceleration.y*self.imu.linear_acceleration.y + self.imu.linear_acceleration.z*self.imu.linear_acceleration.z))

        self.angular_velocity = imu.angular_velocity.z
        self.past_imu = imu
        self.firstimu = 0

        odom_pub = rospy.Publisher('/fix/Odom', Odometry, queue_size=10)
        #odom_broadcaster = tf.TransformBroadcaster()

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        # print(np.arctan2(self.x_k[3].astype(float),self.x_k[2].astype(float)))
        if self.gps is not None:
            odom.pose.pose = Pose(
                Point(self.position.x, self.position.y, 0.0), (self.imu.orientation))
        else:
            odom.pose.pose = Pose(Point(0.0, 0.0, 0.0), (self.imu.orientation))
        odom.child_frame_id = "base_link"
        if self.gps_velocity is not None:
            odom.twist.twist = Twist(
                (self.gps_velocity.vector), Vector3(0., 0., 0.))
        else:
            odom.twist.twist = Twist(Vector3(0., 0., 0.), Vector3(0., 0., 0.))

        # print(self.acc_k[0])
        odom_pub.publish(odom)
    # def mf_callback(self,mf):
        #self.mf = mf
        # self.past_heading.append(self.heading)
        # if self.imu is not None:

        #self.heading = math.atan2(self.mf.magnetic_field.y, self.mf.magnetic_field.x)*(180/math.pi) + self.D
        # print(self.heading)
        # print(imu_heading)
        #self.past_mf = mf
    def gps_velocity_callback(self, gps_velocity):
        self.gps_velocity = gps_velocity
        self.velocity = gps_velocity.vector

    def gps_callback(self, gps):

        self.gps = gps
        self.gpsdt = gps.header.stamp.to_sec() - self.gpstime
        if self.gpsdt > 5:  # Double check this!
            self.gpsdt = 1.0
        self.gpstime = gps.header.stamp.to_sec()
        if self.past_gps is not None:
            if self.firstgps == 0:
                self.firstposition = utm.from_latlon(
                    self.past_gps.latitude, self.past_gps.longitude)
                self.firstgps = 1
            if self.calibrated is True and self.firstgps == 1:
                self.firstposition = utm.from_latlon(
                    self.past_gps.latitude, self.past_gps.longitude)
                self.firstgps = 2

            present = utm.from_latlon(gps.latitude, gps.longitude)
            past = utm.from_latlon(
                self.past_gps.latitude, self.past_gps.longitude)
            self.dx = present[0]-past[0]
            self.dy = present[1]-past[1]

        else:
            self.firstgps = 0
            present = [0, 0]
            past = [0, 0]
            self.dx = 0
            self.dy = 0

        self.position = Vector3(
            self.firstposition[0]-present[0], self.firstposition[1]-present[1], 0.0)
        self.xk = [self.position.x, self.position.y,
                   self.velocity.x, self.velocity.y]

        # self.velocity = gps.twist.twist.linear #Vector3((self.position.x-self.past_position_x[-1])/self.gpsdt, (self.position.y-self.past_position_y[-1])/self.gpsdt, (self.position.z-self.past_position_z[-1])/self.gpsdt)

        # Calculate Heading (Yaw orientation)
        self.yawgps = np.arctan2(
            self.position.y-self.dy, self.position.x-self.dx)
        # if self.firstodom == 1:
        #    self.firstheadingodom = self.yawgps
        #self.headingodom = self.yawgps - self.firstheadingodom
        self.yawgps = self.pi_2_pi(self.yawgps)
        #self.headingodom = self.pi_2_pi(self.headingodom)
        #self.angular_velocity_gps = self.gps.twist.twist.angular.z
        self.past_gps = gps

        self.past_position_x.append(self.position.x)
        self.past_position_y.append(self.position.y)
        self.past_position_z.append(self.position.z)

    def talker(self):
        #pub = rospy.Publisher('/mur/Odom', String, queue_size=10)
        #rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():

            self.kalman()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('VehiclePose')

    server = Server()

    rospy.Subscriber("/piksi/imu", Imu, server.imu_callback)
    #rospy.Subscriber("/android/magnetometer", MagneticField, server.mf_callback)
    rospy.Subscriber("/piksi/gps", NavSatFix, server.gps_callback)
    rospy.Subscriber("/piksi/velocity", Vector3Stamped,
                     server.gps_velocity_callback)

    try:
        server.talker()
    except rospy.ROSInterruptException:
        pass

    # server.telemetry_plot()
    # plt.ion
    # plt.show()

    rospy.spin()
