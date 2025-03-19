 #!/usr/bin/env python
 # license removed for brevity
import rospy
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header, Int8
from cf_cbf.msg import DronePosVelMsg, DroneConstraintMsg, DroneParamsMsg, DroneInt8Array
from tb_cbf.msg import UgvPosVelMsg, UgvConstraintMsg, UgvParamsMsg, UgvInt8Array
import time
import numpy as np
from numpy import linalg as la
import cvxpy
import sys
from cbf_constraints.drone_parameters import DroneParameters
from cbf_constraints.ugv_parameters import UgvParameters

def dist(x_):
    return np.linalg.norm(x_)

def sq_dist(x_, y_):
    return np.sum(np.square(x_) / np.square(y_))


class ConstraintUpdater:
    droneLandSub = []
    droneOdomSub = []
    droneParamSub = []
    droneModeSub = []
    droneParamPub = []
    droneConsPub = []
    droneRefSub = []
    droneModePub = []
    ugvOdomSub = []
    ugvParamSub = []
    ugvParamPub = []
    ugvRefSub = []
    ugvModeSub = []
    ugvConsPub = []
    ugvModePub = []
    def __init__(self, name, no_agents):
        self.name = name
        self.filterFlag = False
        self.no_agents = no_agents

        sq_no = 2
        while sq_no*sq_no < 2*self.no_agents:
            sq_no = sq_no + 1


        self.offsetW = [0.12, 3.65]

        self.t = rospy.get_time()
        self.drones = [DroneParameters('dcf2'), DroneParameters('dcf6'), DroneParameters('cf8')]
        self.ugvs = [UgvParameters('tb1'), UgvParameters('tb3'), UgvParameters('tb4')]

        # for i in range(no_agents):
        #     drone_name = 'dcf' + str(i+1)
        #     ugv_name = 'demo_turtle' + str(i+1)
        #     self.drones.append(DroneParameters(drone_name))
        #     self.ugvs.append(UgvParameters(ugv_name))

        self.droneUpdateModeSub = rospy.Subscriber('/uav_modes', DroneInt8Array, self.setFilter)
        self.rate = rospy.Rate(60)


        for drone in self.drones:
            self.droneOdomSub.append(rospy.Subscriber('/vicon/{}/{}/odom'.format(drone.name, drone.name), Odometry, drone.odom_cb))
            self.droneParamSub.append(rospy.Subscriber('/{}/update_params'.format(drone.name), DroneParamsMsg, drone.params_cb))
            self.droneRefSub.append(rospy.Subscriber('/{}/ref'.format(drone.name), DronePosVelMsg, drone.ref_cb))
            self.droneParamPub.append(rospy.Publisher('/{}/params'.format(drone.name), DroneParamsMsg, queue_size=10))
            self.droneConsPub.append(rospy.Publisher('/{}/cons'.format(drone.name), DroneConstraintMsg, queue_size=10))


        for ugv in self.ugvs:
            self.ugvOdomSub.append(rospy.Subscriber('/vicon/{}/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
            # self.ugvOdomSub.append(rospy.Subscriber('/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
            self.ugvParamSub.append(rospy.Subscriber('/{}/update_params'.format(ugv.name), UgvParamsMsg, ugv.params_cb))
            self.ugvRefSub.append(rospy.Subscriber('/{}/ref'. format(ugv.name), UgvPosVelMsg, ugv.ref_cb))
            self.ugvConsPub.append(rospy.Publisher('/{}/cons'. format(ugv.name), UgvConstraintMsg, queue_size=10))
            self.ugvParamPub.append(rospy.Publisher('/{}/params'.format(ugv.name), UgvParamsMsg, queue_size=10))


        print('Sleeping')
        time.sleep(1)
        print('Awake')


        # maxBoundX = (sq_no*0.5 + 1.5)/2.5
        maxBoundX = 1.2
        minBoundX = -maxBoundX
        minBoundY = -maxBoundX
        maxBoundY = maxBoundX
        maxBoundZ = 1.2
        minBoundZ = 0.0

        self.droneSetpoints = 0.1*np.arange(0,27) - 1.3
        self.A_ = []
        self.b_ = []
        self.C_ = []
        self.d_ = []

        for i in range(self.no_agents):
            ugv = self.ugvs[i]
            drone = self.drones[i]
            rospy.loginfo(f'[constraint_updater]: Checking the odometry for {ugv.name}.', logger_name="constraint_updater")
            try:
                while not ugv.odomFlag:
                    if rospy.is_shutdown():
                        raise SystemError('Node is Shutdown')
                    self.rate.sleep()
            except SystemError:
                print(f'{self.name} is shut down by KeyboardInterrupt.')
                break
            rospy.loginfo(f'[constraint_updater]: Received the odometry for {ugv.name}.', logger_name="constraint_updater")

            ugv.updateBounds([maxBoundX, minBoundX, maxBoundY, minBoundY])
            rospy.loginfo("[constraint_updater]: Updating the bounds for UGV: %s", ugv.name)

            rospy.loginfo(f'[constraint_updater]: Checking the odometry for {drone.name}.')
            try:
                while not drone.odomFlag:
                    if rospy.is_shutdown():
                        raise SystemError('Node is Shutdown')
                    self.rate.sleep()
            except SystemError:
                print(f'{self.name} is shut down by KeyboardInterrupt.')
                break
            rospy.loginfo(f'[constraint_updater]: Received the odometry for {drone.name}.')

            drone.updateBounds([maxBoundX, minBoundX, maxBoundY, minBoundY, maxBoundZ, minBoundZ])
            rospy.loginfo("[constraint_updater]: Updating the bounds for drone: %s", drone.name)
            
            rospy.loginfo(f"[constraint_updater]: Checking the params for drone: {drone.name}")
            try:
                while not drone.paramFlag:
                    if rospy.is_shutdown():
                        raise SystemError('Node is Shutdown')
                    self.rate.sleep()
            except SystemError:
                print(f'{self.name} is shut down by KeyboardInterrupt.')
                break
            
            paramMsg = DroneParamsMsg()
            self.droneParamPub[i].publish(paramMsg)
            rospy.loginfo("[constraint_updater]: Received params for drone: %s", drone.name)
            
            rospy.loginfo("[constraint_updater]: Checking the params for ugv: %s", ugv.name)
            try:
                rospy.loginfo(f'[constraint_updater]: Checking UGV: {ugv.name}\'s parameters.')
                while not ugv.paramFlag:
                    if rospy.is_shutdown():
                        raise SystemError('Node is Shutdown')
                    self.rate.sleep()
            except SystemError:
                print(f'{self.name} is shut down by KeyboardInterrupt.')
                break
            rospy.loginfo(f'[constraint_updater]: Received UGV: {ugv.name}\'s parameters.')

            paramMsg = UgvParamsMsg()
            self.ugvParamPub[i].publish(paramMsg)
            rospy.loginfo("[constraint_updater]: Received params for ugv: %s", ugv.name)


        while not rospy.is_shutdown():
            self.loop()
            self.rate.sleep()

    def updateUgvBoundCons(self, i):
        ugvI = self.ugvs[i]
        self.C_[i][0,0] = -1
        self.C_[i][0,1] =  0
        self.d_[i][0] =  -ugvI.omegaB*(ugvI.maxBoundX + self.offsetW[0] - ugvI.off - ugvI.posOff[0])
        self.C_[i][1,0] = 1
        self.C_[i][1,1] =  0
        self.d_[i][1] =  -ugvI.omegaB*(-ugvI.off + ugvI.posOff[0] - ugvI.minBoundX - self.offsetW[0])
        self.C_[i][2,0] = 0
        self.C_[i][2,1] =  -1
        self.d_[i][2] =  -ugvI.omegaB*(ugvI.maxBoundY + self.offsetW[1] - ugvI.off - ugvI.posOff[1])
        self.C_[i][3,0] = 0
        self.C_[i][3,1] =  1
        self.d_[i][3] =  -ugvI.omegaB*(-ugvI.off + ugvI.posOff[1] - ugvI.minBoundY - self.offsetW[1])


    def updateDroBoundCons(self, i):
        droneI = self.drones[i]
        self.A_[i][0,0] = -1
        self.A_[i][0,1] =  0
        self.b_[i][0] =  -droneI.omegaB*(droneI.maxBoundX + self.offsetW[0] - droneI.pos[0])
        self.A_[i][1,0] = 1
        self.A_[i][1,1] =  0
        self.b_[i][1] =  -droneI.omegaB*(droneI.pos[0] - droneI.minBoundX - self.offsetW[0])
        self.A_[i][2,0] = 0
        self.A_[i][2,1] =  -1
        self.b_[i][2] =  -droneI.omegaB*(droneI.maxBoundY + self.offsetW[1] - droneI.pos[1])
        self.A_[i][3,0] = 0
        self.A_[i][3,1] =  1
        self.b_[i][3] =  -droneI.omegaB*(droneI.pos[1] - droneI.minBoundY - self.offsetW[1])
        self.A_[i][4,0] = 0
        self.A_[i][4,2] =  -1
        self.b_[i][4] =  -droneI.omegaB*(droneI.maxBoundZ - droneI.pos[2])

    def updateLandCons(self, i):
        droneI = self.drones[i]
        ugvI = self.ugvs[i]
        sqHorDist = sq_dist(ugvI.pos[:2] - droneI.pos[:2], np.ones((2,)))
        ugvErrPos = droneI.pos - ugvI.pos
        ugvErrVel = droneI.vel[:2] - ugvI.vel[:2]

        h = ugvErrPos[2] - ugvI.kRate*ugvI.kScaleD*sqHorDist*(np.exp(-ugvI.kRate*sqHorDist)) - ugvI.kOffset
        dhdx = 2*ugvI.kRate*ugvI.kScaleD*(ugvI.kRate*sqHorDist - 1)*np.exp(-ugvI.kRate*sqHorDist)*ugvErrPos[0]
        dhdy = 2*ugvI.kRate*ugvI.kScaleD*(ugvI.kRate*sqHorDist - 1)*np.exp(-ugvI.kRate*sqHorDist)*ugvErrPos[1]
        dhdz = 1
        # dhdt = 2*ugvI.kRate*ugvI.kScaleD*(1 - ugvI.kRate)*np.exp(-ugvI.kRate*sqHorDist)*(ugvErrPos[0]*ugvI.vel[0] + ugvErrPos[1]*ugvI.vel[1])
        dhdt = 0.0

        self.A_[i] = np.vstack((self.A_[i], np.array([dhdx,dhdy, dhdz])))
        self.b_[i] = np.vstack((self.b_[i], - ugvI.omegaD*h - dhdt))


    def updateUgvSphCons(self, i, k):
        ugvI = self.ugvs[i]
        ugvK = self.ugvs[k]
        ugvErrPos = ugvI.posOff - ugvK.posOff
        if la.norm(ugvErrPos) < 2.0:            
            sqHorDist = sq_dist(ugvI.pos[:2] - ugvK.pos[:2], np.array([1,1]))
            rad = ugvI.kRad + ugvI.off
            h = sqHorDist - rad*rad
            self.C_[i] = np.vstack((self.C_[i], np.array([2*ugvErrPos[0], 2*ugvErrPos[1]])))
            self.C_[k] = np.vstack((self.C_[k], -np.array([2*ugvErrPos[0], 2*ugvErrPos[1]])))
            # whdhdt = -ugvK.omegaA*h + 2*ugvK.kScaleA*(ugvErrPos[0]*ugvK.vel[0] + ugvErrPos[1]*ugvK.vel[1])
            # print(whdhdt)
            self.d_[i] = np.vstack((self.d_[i], -ugvK.omegaC*h + 2*(ugvErrPos[0]*ugvK.vel[0] + ugvErrPos[1]*ugvK.vel[1])))
            self.d_[k] = np.vstack((self.d_[k], -ugvK.omegaC*h - 2*(ugvErrPos[0]*ugvI.vel[0] + ugvErrPos[1]*ugvI.vel[1])))

    def updateDroSphCons(self, i, j):
        droneI = self.drones[i]
        droneJ = self.drones[j]
        droErrPos = droneI.pos - droneJ.pos
        # rospy.loginfo(f'[constraint_updater]: Distance Drone {i}: Drone {j} = {dist(droErrPos)}')
        if dist(droErrPos) < 2.0:
            h = sq_dist(droErrPos, droneI.kRad) - 1.0
            scaled_disp = 2*droErrPos/(droneI.kRad*droneI.kRad)

            self.A_[i] = np.vstack((self.A_[i], scaled_disp))
            self.A_[j] = np.vstack((self.A_[j], -scaled_disp))
            self.b_[i] = np.vstack((self.b_[i], -droneI.omegaC*h + scaled_disp@droneJ.vel))
            self.b_[j] = np.vstack((self.b_[j], -droneJ.omegaC*h - scaled_disp@droneI.vel))


    def updateAirCons(self, i, k):
        droneI = self.drones[i]
        ugvK = self.ugvs[k]
        ugvErrPos = droneI.pos - ugvK.pos
        # rospy.loginfo(f'[constraint_updater]: Distance Drone {i}: UGV {k} = {dist(ugvErrPos)}')
        if dist(ugvErrPos) < 2.0:
            sqHorDist = sq_dist(ugvK.pos[:2] - droneI.pos[:2], np.array([1,1]))
            h = ugvErrPos[2] - ugvK.kHeight + ugvK.kScaleA*sqHorDist
            self.A_[i] = np.vstack((self.A_[i], np.array([2*ugvK.kScaleA*ugvErrPos[0], 2*ugvK.kScaleA*ugvErrPos[1], 1])))
            self.b_[i] = np.vstack((self.b_[i], -ugvK.omegaA*h + 2*ugvK.kScaleA*(ugvErrPos[0]*ugvK.vel[0] + ugvErrPos[1]*ugvK.vel[1])))

    def pubUgvConsMatrix(self, index):
        ugvConsMsg = UgvConstraintMsg()        
        try:
            Cd_ = np.hstack((self.C_[index], self.d_[index].reshape((-1,1)))).flatten()
            ugvConsMsg.constraints = Cd_.tolist()
            self.ugvConsPub[index].publish(ugvConsMsg)
        except ValueError:
            print('Incompatible C and d')

    def pubDroConsMatrix(self, index):
        droneConsMsg = DroneConstraintMsg()        
        try:
            Ab_ = np.hstack((self.A_[index], self.b_[index].reshape((-1,1)))).flatten()
            droneConsMsg.constraints = Ab_.tolist()
            self.droneConsPub[index].publish(droneConsMsg)
        except ValueError:
            print('Incompatible A and b')

    def setFilter(self, msg):
        self.filterFlag = True

    def loop(self):
        self.A_ = [np.zeros((5,3))]*self.no_agents
        self.b_ = [np.zeros((5,1))]*self.no_agents
        self.C_ = [np.zeros((4,2))]*self.no_agents
        self.d_ = [np.zeros((4,1))]*self.no_agents

        for i in range(no_agents):
            droneI = self.drones[i]
            ugvI = self.ugvs[i]

            self.updateUgvBoundCons(i)

            for k in range(i+1, no_agents):
                ugvK = self.ugvs[k]
                self.updateUgvSphCons(i, k)

            self.pubUgvConsMatrix(i)

            if self.filterFlag:
                self.updateDroBoundCons(i)
                self.updateLandCons(i)

                for j in range(i+1, no_agents):
                    droneJ = self.drones[j]
                    self.updateDroSphCons(i, j)

                for k in range(no_agents):
                    if i != k:
                        ugvK = self.ugvs[k]
                        self.updateAirCons(i, k)

                self.pubDroConsMatrix(i)




if __name__ == '__main__':
     try:
        rospy.init_node('constraint_updater', anonymous=True)
        # no_agents = int(rospy.get_param('~no_of_agents'))
        no_agents = 3

        dc = ConstraintUpdater('constraint_updater', no_agents)
     except rospy.ROSInterruptException:
        pass
