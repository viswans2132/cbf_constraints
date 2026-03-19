 #!/usr/bin/env python
 # license removed for brevity
import pkg_resources
from tf.transformations import euler_from_quaternion, quaternion_matrix
import time
import numpy as np
import cvxpy as cp
import sys

class DroneParameters(object):
    name = 'crazyflie'

    def __init__(self, name):
        self.name = name

        self.pos = np.array([0.0, 0, 0])
        self.quat = np.array([0.0, 0, 0, 1])
        self.yaw = 0.0
        self.R = np.eye(3)

        self.vel = np.array([0, 0.0, 0])
        self.ang_vel = np.array([0.0, 0, 0])

        self.desPos = np.array([0.0, 0.0, 0.8])
        self.desVel = np.array([0.0, 0, 0])
        self.desYaw = 0.0
        self.desYawVel = 0.0
        self.errVel = np.array([0.0, 0, 0])
        self.errInt = np.array([0.0, 0, 0])
        self.errVelInt = np.array([0.0, 0, 0])
        self.errRel = np.array([[10.0, 10, 10], [10.0, 10, 10]]).T

        self.odomStatus = False
        self.hz = 30.0
        self.dt = 1/self.hz

        self.Kpos = np.array([-3.5, -3.5, -0.7])
        self.kPos = -self.Kpos
        self.Kvel = np.array([-0.5, -0.5, -0.8])
        self.Kder = np.array([-0.05, -0.05, -0.08])
        # self.Kder = np.array([-0.05, -0.05, -0.05])
        self.KintP = np.array([-0.5, -0.5, -0.0])
        self.KintV = np.array([-0.0, -0.0, -0.2])
        # self.KintV = np.array([-0.0, -0.0, -0.05])
        self.Kyaw = 1

        self.kRad = np.array([0.5, 0.5, 1.0])
        self.omegaB = 5.0
        self.omegaC = 1.0

        self.landCounter = 0.0
        self.landFlag = False
        self.startFlag = False
        self.filterFlag = False
        self.followFlag = False
        self.returnFlag = False
        self.secondTask = False
        self.secondTaskDone = False

        self.errorFlag = False
        self.paramFlag = False

        self.landTimerMax = 30
        self.decrement = 0.04

        self.maxInt = np.array([0.1, 0.1, 0.0])
        self.maxVelInt = np.array([0.3, 0.3, 0.5])
        self.maxAcc = np.array([0.3, 0.3, 0.3])

        self.A = np.zeros((3,))
        self.b = 0.0
        self.P = np.eye(3)
        self.u = cp.Variable(3)

        self.droneMode = 0

        self.drone_qp_times = []
        self.ugv_qp_times = []




        print("crazyflie added: {}".format(self.name))

    def setOdom(self, position, quat, velocity):
        self.pos[0] = position[0]
        self.pos[1] = position[1]
        self.pos[2] = position[2]
        self.quat[0] = quat[0]
        self.quat[1] = quat[1]
        self.quat[2] = quat[2]
        self.quat[3] = quat[3]

        if np.absolute(self.quat[1]) > 0.5 or np.absolute(self.quat[0]) > 0.5:
            self.landFlag = True
            self.landTimerMax = 2
            self.decrement = 0.5


        self.yaw = euler_from_quaternion(self.quat)[2]

        R_inv = quaternion_matrix(self.quat)[:-1, :-1]
        # self.R = np.linalg.inv(R_inv)
        self.R = np.array([[np.cos(self.yaw), np.sin(self.yaw), 0], [-np.sin(self.yaw), np.cos(self.yaw), 0], [0, 0, 1]])
        self.vel = R_inv.T.dot(np.array([velocity[0], velocity[1], velocity[2]]))
        # self.vel[0] = velocity[0]
        # self.vel[1] = velocity[1]
        # self.vel[2] = velocity[2]
        self.ang_vel[2] = velocity[3]

        if self.odomStatus == False:
            self.desPos[0] = self.pos[0]
            self.desPos[1] = self.pos[1]
            print('Odometry Received')
        self.odomStatus = True
        # print('odom_received')

    def clearConstraintMatrices(self):
        self.A = np.zeros((3,))
        self.b = 0.0

    def updateConstraintMatrices(self, A_, b_):
        self.A = A_
        self.b = b_
        # print(self.A)
        # print('A: {}'.format(self.A))
        # print('b: {}'.format(self.b))

    def ref_cb(self, msg):
        # print(msg.position)
        try:
            pose = np.array([msg.position[0], msg.position[1], msg.position[2], msg.yaw])
            vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.yawVelocity])
            self.setRef(pose, vel)
        except IndexError:
            print('Ref msg empty: {}'.format(msg.position))


    def odom_cb(self, msg):
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        velocity = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.z])
        self.setOdom(position, quat, velocity)
        # self.timer = rospy.get_time()


    def setMode(self, data):
        if data.data == 0:
            if not self.filterFlag:
                print('filter: ON {}'.format(self.name))
            self.filterFlag = True
            self.followFlag = True
            self.landFlag = False

        elif data.data == 1:
            if self.startFlag and self.filterFlag and self.followFlag:
                self.returnFlag = True
                print('return {}'.format(self.name))
            self.startFlag = True
            print('takeoff {}'.format(self.name))

        elif data.data == 2:
            if self.landFlag == False:
                print('landing {}'.format(self.name))
                if self.pos[2] > 0.5:
                    self.decrement = 0.01
                    self.landTimerMax = 60

                if self.secondTask:
                    self.secondTaskDone = True

                self.secondTask = True


            self.landFlag = True
        self.droneMode = data.data


    def updateBounds(self, bounds):
        self.maxBoundX = bounds[0]
        self.minBoundX = bounds[1]
        self.maxBoundY = bounds[2]
        self.minBoundY = bounds[3]
        self.maxBoundZ = bounds[4]
        self.minBoundZ = bounds[5]

    def filterValues(self, err, u_):
        # print([self.A.shape, self.u.shape])
        # if np.linalg.norm(u_) > 1.0:
        #     u_ = u_*1.0/np.linalg.norm(u_)
        # u_ = np.maximum(-np.array([0.3, 0.3, 0.2]), np.minimum(np.array([0.3, 0.3, 0.5]), u_))
        try:
            constraints = [self.A@self.u >= self.b]
            prob = cp.Problem(cp.Minimize(cp.quad_form(self.u-u_, self.P)), constraints)
            try:
                result = prob.solve()
                desVel = self.u.value
            except cp.error.SolverError:
                print("Solver Error: Holding the position")
                desVel = np.array([0.0, 0.0, 0.0])


        except ValueError:
            print(f"[{self.name}_lib]: Constraint matrices have incompatible dimensions {self.A.shape}:{self.B.shape}")
            self.landFlag = True
            desVel = np.array([0,0,-0.1])

        # desVel = u_

        # if self.name == "dcf6":
        #     print("{:.3f}, {:.3f}, {:.3f}".format(desVel[0], desVel[1], desVel[2]))
        #     print("{:.3f}, {:.3f}, {:.3f}".format(u_[0], u_[1], u_[2]))
        #     pass
        try:
            # print(f'[{self.name}_lib]: Filtered velocity: {desVel}')
            desVel = np.maximum(-np.array([0.3, 0.3, 0.2]), np.minimum(np.array([0.3, 0.3, 0.4]), desVel))
        except TypeError:
            print(f'[{self.name}_lib]: Type error in the filter')
            desVel = np.array([0,0.0,0])

        return desVel


    def setFollow(self, data):
        self.followFlag = True

    def setRef(self, pos, vel):
        if self.followFlag:
            self.desPos = np.array([pos[0], pos[1], pos[2]])
            self.desVel = np.array([vel[1], vel[2], vel[3]])
            self.desYaw = pos[3]
            self.desYawVel = vel[3]

    def publishCmdVel(self, data):
        self.cmd_pub.publish(data)
    
    def odomStatus(self):
        return self.odomStatus

    def generateVelocityInputs(self, velArray):
        desVel2 = np.array([0.0, 0.0, 0.0])
        if self.odomStatus:
            # print("Odometry status is: ".format(self.odomStatus))
            errPos = self.pos - self.desPos
            if self.name == "dcf1":
                print(f'[{self.name}_lib]: Error: {errPos[0]:.2f}: {errPos[1]:.2f}: {errPos[2]:.2f}')
            # if self.name == "dcf2":
            if self.returnFlag and np.linalg.norm(errPos[:2]) < 0.5:
                self.errInt = self.errInt + errPos*self.dt
                self.errInt = np.maximum(-self.maxInt, np.minimum(self.maxInt, self.errInt))

            desVel2 = self.Kpos * errPos + self.KintP * self.errInt + self.desVel
            # print(f'[{self.name}_lib]: Velocity before filtering: {desVel2}')

            if self.filterFlag:
                desVel2 = self.filterValues(errPos, desVel2)
            # print(f'[{self.name}_lib]: Velocity after filtering: {desVel2}')

        if self.landFlag:
            if self.startFlag: 
                print('landing: {}'.format(self.name))
            self.startFlag = False
            self.filterFlag = 0.0
            velArray[2] = -2.5
            velArray[3] = 0.0


        else:
            velArray[0] = desVel2[0]
            velArray[1] = desVel2[1]
            velArray[2] = desVel2[2]
        # print(f'[{self.name}_lib]: Velocity: {velArray}')
        return self.odomStatus



    def generateControlInputs(self, velArray):
        uPitch = 0.0
        uRoll = 0.0
        uThrust = 0.0
        uYaw = 0.0
        if self.odomStatus:
            # print("Odometry status is: ".format(self.odomStatus))
            errPos = self.pos - self.desPos
            # if self.name == "dcf2":
            # print('{}: Error: {:.3f}, {:.3f}, {:.3f}'.format(self.name, errPos[0], errPos[1], errPos[2]))
            if self.returnFlag and np.linalg.norm(errPos[:2]) < 0.5:
                self.errInt = self.errInt + errPos*self.dt
                self.errInt = np.maximum(-self.maxInt, np.minimum(self.maxInt, self.errInt))
                # print('{}: Error: {:.3f}, {:.3f}, {:.3f}'.format(self.name, errInt[0], errInt[1], errInt[2]))

            desVel2 = self.Kpos * errPos + self.KintP * self.errInt + self.desVel

            if self.filterFlag:
                desVel2 = self.filterValues(errPos, desVel2)


            derVel = ((self.vel - desVel2) - self.errVel)/self.dt
            self.errVel = self.vel - desVel2

            if self.startFlag:
                self.errVelInt = self.errVelInt + self.errVel*self.dt
            self.errVelInt = np.maximum(-self.maxVelInt, np.minimum(self.maxVelInt, self.errVelInt))
            # print(self.errVelInt)

            des_a = self.Kvel * self.errVel + self.Kder * derVel + self.KintV * self.errVelInt
            # des_a = self.Kvel * self.errVel
            des_a = self.R.dot(des_a)
            # print(des_a)
            # print(des_a[3])
            des_a = np.maximum(-self.maxAcc, np.minimum(self.maxAcc, des_a))
            # print("Error: {0:.3f}: {1:.3f}: {2:.3f}: \n Acc: {3:.3f}: {4:.3f}: {5:.3f}".format(self.errVel[0], errPos[1], errPos[2], des_a[0], des_a[1], des_a[2]))
            # print("{:.3f}: {:.3f}: {:.3f}".format(errPos[0], errPos[1], errPos[2]))

            # yaw_diff = np.minimum(0.2, np.maximum(self.desYaw - self.yaw, -0.2))

            yaw_des = -1.0*self.yaw -0.5*self.ang_vel[2]


            uPitch = des_a[0]
            uRoll = des_a[1]
            uYaw = yaw_des

            if self.landFlag:
                self.startFlag = False
                self.filterFlag = False
                # print('{}: Landing')
                uThrust = 0.54 - self.landCounter*self.decrement
                uRoll = 0.0
                uPitch = 0.0
                if self.landCounter > self.landTimerMax:
                    uThrust = 0.0
                    uyaw = 0.0

                self.landCounter =  self.landCounter + 1

            elif self.startFlag:
                uThrust = des_a[2] + 0.63
            # print("thrust: {}".format(uThrust))
        else:
            # print("{}: Odometry not received".format(self.name))
            pass

        velArray[0] = uPitch
        velArray[1] = uRoll
        velArray[2] = uYaw
        velArray[3] = uThrust

        # print("{}: {:.3f}: {:.3f}: {:.3f}: {:.3f}: Ready to publish".format(self.name, uPitch, uRoll, uYaw, uThrust))
        return self.odomStatus


        # self.cmdPub.publish(velMsg)