import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_matrix
from cf_cbf.msg import DroneParamsMsg, DronePosVelMsg
from nav_msgs.msg import Odometry

class DroneParameters(object):
    name = 'drone_param'
    def __init__(self, name):    
        self.name = name
        self.pos = np.array([0.0, 0, 0])
        self.desPos = np.array([0.0, 0, 0])
        self.desPos1 = np.array([0.0, 0, 0])
        self.desPos2 = np.array([0.0, 0, 0])
        self.desVel = np.array([0.0, 0, 0])
        self.quat = np.array([0.0, 0, 0, 1])
        self.yaw = 0.0
        self.offsetAngle = 0.0
        self.R = np.eye(3)

        self.vel = np.array([0, 0.0, 0])
        self.ang_vel = np.array([0.0, 0, 0])

        self.kRad = np.array([0.3, 0.3, 0.8])
        self.omegaC = 5.0
        self.omegaB = 1.0

        self.odomFlag = False
        self.followFlag = False
        self.returnFlag = False
        self.firstTask = True
        self.secondTask = False
        self.landTime = rospy.get_time()

        self.startTime = rospy.get_time()

        self.droneMode = 0
        self.taskFlag = False
        self.paramFlag = False

        self.maxBoundX = 1.5
        self.southBound = -1.5
        self.eastBound = -1.5
        self.westBound = -1.5


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
        self.paramFlag =  True
        # paramsPublisher = rospy.Publisher('/{}/params'.format(self.name), DroneParamsMsg, queue_size=10)
        # paramsPublisher.publish(msg)
        # print('Parameters received: {}'.format(self.name))


    def land_cb(self, msg):
        # if self.firstTask:
        #     self.firstTask = False
        #     self.secondTask = True
        #     self.returnFlag = False
        #     self.followFlag = True
        #     self.landTime = rospy.get_time()
        if self.firstTask:
            self.firstTask = False
            self.taskFlag = False
            self.returnFlag = False
            self.followFlag = True
            self.landTime = rospy.get_time()



    def mode_cb(self, msg):
        self.droneMode = msg.data


    def updateBounds(self, bounds):
        self.maxBoundX = bounds[0]
        self.minBoundX = bounds[1]
        self.maxBoundY = bounds[2]
        self.minBoundY = bounds[3]
        self.maxBoundZ = bounds[4]
        self.minBoundZ = bounds[5]

    def ref_cb(self, msg):        
        # print(msg.position)
        try:
            self.desPos = np.array([msg.position[0], msg.position[1], msg.position[2]])
            self.desVel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
        except IndexError:
            print('Ref msg empty: {}'.format(msg.position))