import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_matrix
from tb_cbf.msg import UgvParamsMsg
from nav_msgs.msg import Odometry

class UgvParameters(object):
    name = 'drone_param'
    def __init__(self, name): 
        self.name = name
        self.pos = np.array([0.0, 0, 0])
        self.posOff = np.array([0.0, 0, 0])
        self.quat = np.array([0.0, 0, 0, 1])

        self.vel = np.array([0, 0.0, 0])
        self.ang_vel = np.array([0.0, 0, 0])
        self.offsetAngle = 0.0

        self.off = 0.1

        self.desPos = np.array([2.0, 2.0, 0.0])

        self.kPos = np.array([-2.0, -2.0])


        self.hz = 1.0
        self.dt = 1/self.hz
        self.control_input = np.array([0.0, 0.0, 0.0])

        self.rate = rospy.Rate(self.hz)

        # self.cmd_pub = rospy.Publisher('/{}/cmd_vel'.format(name), TwistStamped, queue_size=1)
        # self.ref_pub = rospy.Publisher('/{}/ref'.format(name), PoseStamped, queue_size=1)

        # self.odom_sub = rospy.Subscriber('/vicon/{}/{}/odom'.format(name, name), Odometry, self.odom_cb)
        # self.cmd_sub = rospy.Subscriber('/old_cmd_vel', TwistStamped, self.oldControl_cb)

        ezp = 0.5
        theta = np.deg2rad(30)
        d = ezp*np.tan(theta)

        self.kScaleD = np.exp(1)*ezp
        self.kRate = 1/(d*d)
        # print([self.kScaleD, self.kRate])
        self.kOffset = 0.0
        self.omegaD = 0.7
        
        self.kRad = 0.2
        self.omegaC = 0.2

        self.kHeight = 1.0
        self.kScaleA = self.kHeight/(self.kRad*self.kRad)
        # print(self.kScaleA)
        self.omegaA = 3.0
        self.omegaB = 0.3


        self.odomFlag = False
        self.constraintsReceived = False

        self.followFlag = True
        self.returnFlag = False
        self.reverseFlag = False

        self.ugvMode = 0


    def odom_cb(self, data):
        self.pos[0] = float(data.pose.pose.position.x)
        self.pos[1] = float(data.pose.pose.position.y) 
        self.pos[2] = float(data.pose.pose.position.z) + 0.02
        self.quat[0] = float(data.pose.pose.orientation.x)
        self.quat[1] = float(data.pose.pose.orientation.y)
        self.quat[2] = float(data.pose.pose.orientation.z)
        self.quat[3] = float(data.pose.pose.orientation.w)

        self.yaw = euler_from_quaternion(self.quat)[2]

        R_inv = quaternion_matrix(self.quat)[:-1, :-1]
        # self.R = np.linalg.inv(R_inv)
        self.R = np.array([[np.cos(self.yaw), np.sin(self.yaw)], [-np.sin(self.yaw), np.cos(self.yaw)]])

        self.posOff[0] = self.pos[0] + np.cos(self.yaw)*self.off
        self.posOff[1] = self.pos[1] + np.sin(self.yaw)*self.off

        self.pos[0] = self.pos[0] - 0.07*np.cos(self.yaw)
        self.pos[1] = self.pos[1] - 0.07*np.sin(self.yaw)
        self.pos[2] = self.pos[2] + 0.25

        velocity = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y])
        self.vel[:2] = self.R.T.dot(velocity)
        self.ang_vel[2] = data.twist.twist.angular.z
        if self.odomFlag == False:
            self.odomFlag = True
            print('Odometry Received: {}'.format(self.name))

    def params_cb(self, msg):
        self.kRad = np.array(msg.kRad)
        self.kOffset = np.array(msg.kOffset)
        self.kRate = np.array(msg.kRate)
        self.kHeight = np.array(msg.kHeight)
        self.kScaleA = np.array(msg.kScaleA)
        self.kScaleD = np.array(msg.kScaleD)
        self.omegaA = msg.omegaA
        self.omegaB = msg.omegaB
        self.omegaC = msg.omegaC
        self.omegaD = msg.omegaD

        paramsPublisher = rospy.Publisher('/{}/params'.format(self.name), UgvParamsMsg, queue_size=10)
        paramsPublisher.publish(msg)
        print('Parameters received: {}'.format(self.name))



    def mode_cb(self, msg):
        self.ugvMode = msg.data