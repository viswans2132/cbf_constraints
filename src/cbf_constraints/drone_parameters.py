import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_matrix
from cf_cbf.msg import DroneParamsMsg
from nav_msgs.msg import Odometry

class DroneParameters(object):
    name = 'drone_param'
    def __init__(self, name):    
        self.name = name
        self.pos = np.array([0.0, 0, 0])
        self.quat = np.array([0.0, 0, 0, 1])
        self.yaw = 0.0
        self.offsetAngle = 0.0
        self.R = np.eye(3)

        self.vel = np.array([0, 0.0, 0])
        self.ang_vel = np.array([0.0, 0, 0])

        self.kRad = np.array([0.3, 0.3, 0.8])
        self.omegaC = 5.0

        self.odomFlag = False
        self.followFlag = False
        self.returnFlag = False

        self.startTime = rospy.get_time()


    def odom_cb(self, data):
        self.pos[0] = float(data.pose.pose.position.x)
        self.pos[1] = float(data.pose.pose.position.y)
        self.pos[2] = float(data.pose.pose.position.z)
        self.quat[0] = float(data.pose.pose.orientation.x)
        self.quat[1] = float(data.pose.pose.orientation.y)
        self.quat[2] = float(data.pose.pose.orientation.z)
        self.quat[3] = float(data.pose.pose.orientation.w)

        self.yaw = euler_from_quaternion(self.quat)[2]

        R_inv = quaternion_matrix(self.quat)[:-1, :-1]
        # self.R = np.linalg.inv(R_inv)
        self.R = np.array([[np.cos(self.yaw), np.sin(self.yaw), 0], [-np.sin(self.yaw), np.cos(self.yaw), 0], [0, 0, 1]])

        velocity = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
        self.vel = self.R.T.dot(velocity)
        self.ang_vel[2] = data.twist.twist.angular.z
        if self.odomFlag == False:
            self.offsetAngle = np.arctan2(self.pos[1], self.pos[0])
            self.odomFlag = True

    def params_cb(self, msg):
        self.kRad = np.array(msg.kRad)
        self.omegaC = msg.omegaC

