 #!/usr/bin/env python
 # license removed for brevity
import rospy
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from std_msgs.msg import Header, Int8
from cf_cbf.msg import DronePosVelMsg, DroneConstraintMsg, DroneParamsMsg, DroneInt8Array
from tb_cbf.msg import UgvPosVelMsg, UgvConstraintMsg, UgvParamsMsg, UgvInt8Array
import time
import numpy as np
from numpy import linalg as la
import cvxpy
import sys
import os
import rospkg
from cbf_constraints.drone_parameters_central import DroneParameters
from cbf_constraints.ugv_parameters_central import UgvParameters

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
    droneCmdPub = []
    droneRefSub = []
    droneModePub = []
    landSignal = []
    ugvOdomSub = []
    ugvParamSub = []
    ugvParamPub = []
    ugvRefSub = []
    ugvModeSub = []
    ugvCmdPub = []
    ugvModePub = []
    def __init__(self, name, no_agents):
        self.name = name
        self.filterFlag = False
        self.no_agents = no_agents

        sq_no = 2
        while sq_no*sq_no < 2*self.no_agents:
            sq_no = sq_no + 1


        self.offsetW = [0.0, -0.0]

        self.t = rospy.get_time()
        self.drones = []
        self.ugvs = []

        # --------------------------------------------------
        # CBF value histories
        # # --------------------------------------------------
        # self.drone_boundary_h_hist = []   # each entry: (N, 5)
        # self.ugv_boundary_h_hist = []     # each entry: (N, 4)
        # self.landing_h_hist = []          # each entry: (N,)
        # self.drone_pair_h_hist = []       # each entry: (N, N)
        # self.ugv_pair_h_hist = []         # each entry: (N, N)
        # self.air_h_hist = []              # each entry: (N, N)


        self.drone_qp_times = []
        self.ugv_qp_times = []
        self.drone_h_logs = []
        self.ugv_h_logs = []
        self.filesSaved = False



        for i in range(no_agents):
            drone_name = 'dcf' + str(i+1)
            ugv_name = 'demo_turtle' + str(i+1)
            self.drones.append(DroneParameters(drone_name))
            self.ugvs.append(UgvParameters(ugv_name))

        self.droneUpdateModeSub = rospy.Subscriber('/uav_modes', DroneInt8Array, self.setFilter)
        self.rate = rospy.Rate(60)


        for drone in self.drones:
            self.droneOdomSub.append(rospy.Subscriber('/{}/odometry_sensor1/odometry'.format(drone.name), Odometry, drone.odom_cb))
            # self.droneParamSub.append(rospy.Subscriber('/{}/update_params'.format(drone.name), DroneParamsMsg, drone.params_cb))
            self.droneModeSub.append(rospy.Subscriber('/{}/update_uav_mode'.format(drone.name), Int8, drone.setMode))
            self.droneRefSub.append(rospy.Subscriber('/{}/ref'.format(drone.name), DronePosVelMsg, drone.ref_cb))
            self.droneParamPub.append(rospy.Publisher('/{}/params'.format(drone.name), DroneParamsMsg, queue_size=10))
            self.droneModePub.append(rospy.Publisher('/{}/uav_mode'.format(drone.name), Int8, queue_size=10))
            # self.droneConsPub.append(rospy.Publisher('/{}/cons1'.format(drone.name), DroneConstraintMsg, queue_size=10))
            self.droneCmdPub.append(rospy.Publisher('/{}/vel_msg1'.format(drone.name), TwistStamped, queue_size=10))
            self.landSignal.append(rospy.Publisher('/{}/land_signal'.format(drone.name), Int8, queue_size=10))



        for ugv in self.ugvs:
            # self.ugvOdomSub.append(rospy.Subscriber('/vicon/{}/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
            self.ugvOdomSub.append(rospy.Subscriber('/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
            # self.ugvParamSub.append(rospy.Subscriber('/{}/update_params'.format(ugv.name), UgvParamsMsg, ugv.params_cb))
            self.ugvModeSub.append(rospy.Subscriber('/{}/update_ugv_mode'.format(ugv.name), Int8, ugv.setMode))
            self.ugvRefSub.append(rospy.Subscriber('/{}/ref'. format(ugv.name), UgvPosVelMsg, ugv.ref_pv_cb))
            self.ugvCmdPub.append(rospy.Publisher('/{}/cmd_vel1'.format(ugv.name), Twist, queue_size=10))
            self.ugvModePub.append(rospy.Publisher('/{}/ugv_mode'.format(ugv.name), Int8, queue_size=10))
            # self.ugvConsPub.append(rospy.Publisher('/{}/cons1'. format(ugv.name), UgvConstraintMsg, queue_size=10))
            self.ugvParamPub.append(rospy.Publisher('/{}/params'.format(ugv.name), UgvParamsMsg, queue_size=10))


        print('Sleeping')
        time.sleep(1)
        print('Awake')


        maxBoundX = sq_no*0.5 + 1.5
        minBoundX = -maxBoundX
        minBoundY = -maxBoundX
        maxBoundY = maxBoundX
        maxBoundZ = 2.0
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
                while not ugv.odomStatus:
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
                while not drone.odomStatus:
                    if rospy.is_shutdown():
                        raise SystemError('Node is Shutdown')
                    self.rate.sleep()
            except SystemError:
                print(f'{self.name} is shut down by KeyboardInterrupt.')
                break
            rospy.loginfo(f'[constraint_updater]: Received the odometry for {drone.name}.')

            drone.updateBounds([maxBoundX, minBoundX, maxBoundY, minBoundY, maxBoundZ, minBoundZ])
            rospy.loginfo("[constraint_updater]: Updating the bounds for drone: %s", drone.name)
            
            # rospy.loginfo(f"[constraint_updater]: Checking the params for drone: {drone.name}")
            # try:
            #     while not drone.paramFlag:
            #         if rospy.is_shutdown():
            #             raise SystemError('Node is Shutdown')
            #         self.rate.sleep()
            # except SystemError:
            #     print(f'{self.name} is shut down by KeyboardInterrupt.')
            #     break
            
            paramMsg = DroneParamsMsg()
            self.droneParamPub[i].publish(paramMsg)
            rospy.loginfo("[constraint_updater]: Received params for drone: %s", drone.name)
            
            # rospy.loginfo("[constraint_updater]: Checking the params for ugv: %s", ugv.name)
            # try:
            #     rospy.loginfo(f'[constraint_updater]: Checking UGV: {ugv.name}\'s parameters.')
            #     while not ugv.paramFlag:
            #         if rospy.is_shutdown():
            #             raise SystemError('Node is Shutdown')
            #         self.rate.sleep()
            # except SystemError:
            #     print(f'{self.name} is shut down by KeyboardInterrupt.')
            #     break
            # rospy.loginfo(f'[constraint_updater]: Received UGV: {ugv.name}\'s parameters.')

            paramMsg = UgvParamsMsg()
            self.ugvParamPub[i].publish(paramMsg)
            rospy.loginfo("[constraint_updater]: Received params for ugv: %s", ugv.name)


        while not rospy.is_shutdown():
            self.loop()           

            if self.check_experiment_end() and not self.filesSaved:
                rospy.loginfo("Experiment completed.")
                self.save_qp_times()
                self.save_h_func()

                self.filesSaved = True

                # break

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
        dhdt = 2*ugvI.kRate*ugvI.kScaleD*(1 - ugvI.kRate)*np.exp(-ugvI.kRate*sqHorDist)*(ugvErrPos[0]*ugvI.vel[0] + ugvErrPos[1]*ugvI.vel[1])

        self.A_[i] = np.vstack((self.A_[i], np.array([dhdx,dhdy, dhdz])))
        self.b_[i] = np.vstack((self.b_[i], - ugvI.omegaD*h - dhdt))


    def updateUgvSphCons(self, i, k):
        ugvI = self.ugvs[i]
        ugvK = self.ugvs[k]
        ugvErrPos = ugvI.posOff - ugvK.posOff
        if la.norm(ugvErrPos) < 2.0:            
            sqHorDist = sq_dist(ugvI.pos[:2] - ugvK.pos[:2], np.array([1,1]))
            rad = ugvI.kRad + 2*ugvI.off
            h = sqHorDist - rad*rad
            self.C_[i] = np.vstack((self.C_[i], np.array([2*ugvErrPos[0], 2*ugvErrPos[1]])))
            self.C_[k] = np.vstack((self.C_[k], -np.array([2*ugvErrPos[0], 2*ugvErrPos[1]])))
            # whdhdt = -ugvK.omegaA*h + 2*ugvK.kScaleA*(ugvErrPos[0]*ugvK.vel[0] + ugvErrPos[1]*ugvK.vel[1])
            # print(whdhdt)
            self.d_[i] = np.vstack((self.d_[i], -ugvK.omegaC*h))
            self.d_[k] = np.vstack((self.d_[k], -ugvK.omegaC*h))

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
            self.b_[i] = np.vstack((self.b_[i], -droneI.omegaC*h))
            self.b_[j] = np.vstack((self.b_[j], -droneJ.omegaC*h))


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



    def build_drone_constraints_centralized(self):
        """
        Build centralized drone CBF constraints:

            A_d @ u_d >= b_d

        where
            u_d = [u_d1; u_d2; ...; u_dN] in R^(3N)

        Returns
        -------
        A_d : np.ndarray, shape (m_d, 3N)
        b_d : np.ndarray, shape (m_d, 1)
        """
        A_rows = []
        b_rows = []
        h_log = {"t": rospy.get_time(), "boundary": np.zeros((self.no_agents, 5)), "landing": np.zeros(self.no_agents), "air": [], "pair": [] }

        nvars = 3 * self.no_agents

        for i in range(self.no_agents):
            droneI = self.drones[i]            
            if droneI.landFlag:
                continue

            ugvI = self.ugvs[i]

            base = 3 * i

            # --------------------------------------------------
            # 1) Drone boundary constraints
            # --------------------------------------------------
            h1 = droneI.maxBoundX + self.offsetW[0] - droneI.pos[0]
            row = np.zeros(nvars)
            row[base + 0] = -1.0
            A_rows.append(row)
            b_rows.append(-droneI.omegaB * h1)

            h2 = droneI.pos[0] - droneI.minBoundX - self.offsetW[0]
            row = np.zeros(nvars)
            row[base + 0] = 1.0
            A_rows.append(row)
            b_rows.append(-droneI.omegaB * h2)

            h3 = (droneI.maxBoundY + self.offsetW[1] - droneI.pos[1])
            row = np.zeros(nvars)
            row[base + 1] = -1.0
            A_rows.append(row)
            b_rows.append(-droneI.omegaB * h3)

            h4 = droneI.pos[1] - droneI.minBoundY - self.offsetW[1]
            row = np.zeros(nvars)
            row[base + 1] = 1.0
            A_rows.append(row)
            b_rows.append(-droneI.omegaB * h4)

            h5 = droneI.maxBoundZ - droneI.pos[2]
            row = np.zeros(nvars)
            row[base + 2] = -1.0
            A_rows.append(row)
            b_rows.append(-droneI.omegaB * h5)


            h_log["boundary"][i, 0] = h1
            h_log["boundary"][i, 1] = h2
            h_log["boundary"][i, 2] = h3
            h_log["boundary"][i, 3] = h4
            h_log["boundary"][i, 4] = h5

            # --------------------------------------------------
            # 2) Landing constraint: drone i relative to its own UGV i
            #
            # Only drone velocity is optimized here, so UGV velocity
            # remains exogenous and stays in b.
            # --------------------------------------------------
            sqHorDist = sq_dist(ugvI.pos[:2] - droneI.pos[:2], np.ones((2,)))
            ugvErrPos = droneI.pos - ugvI.pos

            h = ugvErrPos[2] \
                - ugvI.kRate * ugvI.kScaleD * sqHorDist * np.exp(-ugvI.kRate * sqHorDist) \
                - ugvI.kOffset

            h_log["landing"][i] = h

            dhdx = 2 * ugvI.kRate * ugvI.kScaleD * (ugvI.kRate * sqHorDist - 1) \
                   * np.exp(-ugvI.kRate * sqHorDist) * ugvErrPos[0]
            dhdy = 2 * ugvI.kRate * ugvI.kScaleD * (ugvI.kRate * sqHorDist - 1) \
                   * np.exp(-ugvI.kRate * sqHorDist) * ugvErrPos[1]
            dhdz = 1.0

            # known/exogenous term due to UGV motion
            dhdt = 2 * ugvI.kRate * ugvI.kScaleD * (1 - ugvI.kRate) \
                   * np.exp(-ugvI.kRate * sqHorDist) \
                   * (ugvErrPos[0] * ugvI.vel[0] + ugvErrPos[1] * ugvI.vel[1])

            row = np.zeros(nvars)
            row[base:base+3] = np.array([dhdx, dhdy, dhdz])
            A_rows.append(row)
            b_rows.append(-ugvI.omegaD * h - dhdt)

            # --------------------------------------------------
            # 3) Drone-vs-UGV air constraints
            #
            # Drone optimizer only: UGV velocity is exogenous.
            # --------------------------------------------------
            for k in range(self.no_agents):
                if i == k:
                    continue

                ugvK = self.ugvs[k]
                ugvErrPos = droneI.pos - ugvK.pos

                if dist(ugvErrPos) < 2.0:
                    sqHorDist = sq_dist(ugvK.pos[:2] - droneI.pos[:2], np.array([1.0, 1.0]))
                    h = ugvErrPos[2] - ugvK.kHeight + ugvK.kScaleA * sqHorDist

                    h_log["air"].append({"drone": i, "ugv": k, "h": h })

                    row = np.zeros(nvars)
                    row[base:base+3] = np.array([
                        2 * ugvK.kScaleA * ugvErrPos[0],
                        2 * ugvK.kScaleA * ugvErrPos[1],
                        1.0
                    ])
                    A_rows.append(row)
                    b_rows.append(
                        -ugvK.omegaA * h
                        + 2 * ugvK.kScaleA * (ugvErrPos[0] * ugvK.vel[0] + ugvErrPos[1] * ugvK.vel[1])
                    )

        # ------------------------------------------------------
        # 4) Drone-drone pairwise constraints
        #
        # Centralized form: both drone velocities are decision vars,
        # so both appear in A_d, and only -omega*h stays in b_d.
        # ------------------------------------------------------
        for i in range(self.no_agents):
            droneI = self.drones[i]
            if droneI.landFlag:
                continue
            for j in range(i + 1, self.no_agents):
                droneJ = self.drones[j]
                if droneJ.landFlag:
                    continue

                droErrPos = droneI.pos - droneJ.pos

                if dist(droErrPos) < 2.0:
                    h = sq_dist(droErrPos, droneI.kRad) - 1.0
                    h_log["pair"].append({ "i": i, "j": j, "h": h })
                    scaled_disp = 2 * droErrPos / (droneI.kRad * droneI.kRad)

                    row = np.zeros(nvars)
                    row[3*i:3*i+3] = scaled_disp
                    row[3*j:3*j+3] = -scaled_disp

                    A_rows.append(row)
                    b_rows.append(-droneI.omegaC * h)

        if len(A_rows) == 0:
            A_d = np.zeros((0, nvars))
            b_d = np.zeros((0, 1))
        else:
            A_d = np.array(A_rows, dtype=float)
            b_d = np.array(b_rows, dtype=float).reshape(-1, 1)

        self.drone_h_logs.append(h_log)

        return A_d, b_d


    def build_ugv_constraints_centralized(self):
        """
        Build centralized UGV CBF constraints:

            A_g @ u_g >= b_g

        where
            u_g = [u_g1; u_g2; ...; u_gN] in R^(2N)

        Returns
        -------
        A_g : np.ndarray, shape (m_g, 2N)
        b_g : np.ndarray, shape (m_g, 1)
        """
        A_rows = []
        b_rows = []
        h_log = {"t": rospy.get_time(), "boundary": np.zeros((self.no_agents, 4)), "pair": []}

        nvars = 2 * self.no_agents

        for i in range(self.no_agents):
            ugvI = self.ugvs[i]
            base = 2 * i

            # --------------------------------------------------
            # 1) UGV boundary constraints
            # --------------------------------------------------
            h1 = ugvI.maxBoundX + self.offsetW[0] - ugvI.off - ugvI.posOff[0]
            row = np.zeros(nvars)
            row[base + 0] = -1.0
            A_rows.append(row)
            b_rows.append(-ugvI.omegaB * h1)

            h2 = -ugvI.off + ugvI.posOff[0] - ugvI.minBoundX - self.offsetW[0]
            row = np.zeros(nvars)
            row[base + 0] = 1.0
            A_rows.append(row)
            b_rows.append(-ugvI.omegaB * h2)

            h3 = ugvI.maxBoundY + self.offsetW[1] - ugvI.off - ugvI.posOff[1]
            row = np.zeros(nvars)
            row[base + 1] = -1.0
            A_rows.append(row)
            b_rows.append(-ugvI.omegaB * h3)

            h4 = -ugvI.off + ugvI.posOff[1] - ugvI.minBoundY - self.offsetW[1]
            row = np.zeros(nvars)
            row[base + 1] = 1.0
            A_rows.append(row)
            b_rows.append(-ugvI.omegaB * h4)


            h_log["boundary"][i, 0] = h1
            h_log["boundary"][i, 1] = h2
            h_log["boundary"][i, 2] = h3
            h_log["boundary"][i, 3] = h4

        # ------------------------------------------------------
        # 2) UGV-UGV pairwise constraints
        #
        # Centralized form: both UGV velocities are decision vars.
        # ------------------------------------------------------
        for i in range(self.no_agents):
            for k in range(i + 1, self.no_agents):
                ugvI = self.ugvs[i]
                ugvK = self.ugvs[k]

                ugvErrPos = ugvI.posOff - ugvK.posOff

                if la.norm(ugvErrPos) < 2.0:
                    sqHorDist = sq_dist(ugvI.pos[:2] - ugvK.pos[:2], np.array([1.0, 1.0]))
                    rad = ugvI.kRad + 2 * ugvI.off
                    h = sqHorDist - rad * rad
                    h_log["pair"].append({ "i": i, "k": k, "h": h})

                    grad = np.array([2 * ugvErrPos[0], 2 * ugvErrPos[1]])

                    row = np.zeros(nvars)
                    row[2*i:2*i+2] = grad
                    row[2*k:2*k+2] = -grad

                    A_rows.append(row)
                    b_rows.append(-ugvI.omegaC * h)

        if len(A_rows) == 0:
            A_g = np.zeros((0, nvars))
            b_g = np.zeros((0, 1))
        else:
            A_g = np.array(A_rows, dtype=float)
            b_g = np.array(b_rows, dtype=float).reshape(-1, 1)

        self.ugv_h_logs.append(h_log)
        return A_g, b_g

    def solve_centralized_qps(self):
        A_d, b_d = self.build_drone_constraints_centralized()
        A_g, b_g = self.build_ugv_constraints_centralized()

        u_d_ref, u_g_ref = self.build_nominal_references()

        u_d = cvxpy.Variable(3 * self.no_agents)
        u_g = cvxpy.Variable(2 * self.no_agents)

        drone_obj = cvxpy.Minimize(cvxpy.sum_squares(u_d - u_d_ref))
        ugv_obj = cvxpy.Minimize(cvxpy.sum_squares(u_g - u_g_ref))

        drone_constraints = []
        ugv_constraints = []

        if A_d.shape[0] > 0:
            drone_constraints.append(A_d @ u_d >= b_d.flatten())

        if A_g.shape[0] > 0:
            ugv_constraints.append(A_g @ u_g >= b_g.flatten())

        drone_prob = cvxpy.Problem(drone_obj, drone_constraints)
        ugv_prob = cvxpy.Problem(ugv_obj, ugv_constraints)

        drone_t0 = time.perf_counter()        
        drone_prob.solve()
        drone_t1 = time.perf_counter()
        drone_solve_time = drone_t1 - drone_t0
        self.drone_qp_times.append(drone_solve_time)

        ugv_t0 = time.perf_counter()
        ugv_prob.solve()
        ugv_t1 = time.perf_counter()
        ugv_solve_time = ugv_t1 - ugv_t0
        self.ugv_qp_times.append(ugv_solve_time)



        if drone_prob.status in ["optimal", "optimal_inaccurate"]:
            u_d_fil = np.clip(u_d.value, -0.3, 0.3)
        else:
            # rospy.logwarn(f"QP failed with status: {prob.status}")
            u_d_fil = np.zeros(u_d.shape)

        if ugv_prob.status in ["optimal", "optimal_inaccurate"]:
            u_g_fil = np.clip(u_g.value, -0.15, 0.15)
        else:
            # rospy.logwarn(f"QP failed with status: {prob.status}")
            u_g_fil = np.zeros(u_g.shape)

        return u_d_fil, u_g_fil, u_d_ref, u_g_ref, A_d, b_d, A_g, b_g
    
    def build_nominal_references(self):
        """
        Build stacked nominal references for centralized QPs.

        Drones:
            u_d_nom = Kpos * (p_ref - p)

        UGVs:
            u_g_nom = Kpos * (p_ref_virtual - p_virtual)

        Returns
        -------
        u_d_ref : np.ndarray, shape (3*self.no_agents,)
        u_g_ref : np.ndarray, shape (2*self.no_agents,)
        """
        u_d_ref = np.zeros(3 * self.no_agents)
        u_g_ref = np.zeros(2 * self.no_agents)

        for i in range(self.no_agents):
            drone = self.drones[i]
            ugv = self.ugvs[i]

            # --------------------------------------------------
            # Drone nominal reference
            # --------------------------------------------------
            # Adjust these field names if your ref is stored differently
            drone_pos_err = drone.desPos - drone.pos
            drone_u_nom = drone.kPos * drone_pos_err

            u_d_ref[3*i:3*i+3] = drone_u_nom

            # --------------------------------------------------
            # UGV nominal reference in virtual-point coordinates
            # --------------------------------------------------
            # Current virtual point
            ugv_virtual_pos = np.array([
                ugv.pos[0] + ugv.off * np.cos(ugv.yaw),
                ugv.pos[1] + ugv.off * np.sin(ugv.yaw)
            ])

            # Desired virtual point
            # If your reference is already for the virtual point, use it directly.
            # Here I assume ugv.desPos[:2] is the desired planar position.
            ugv_ref_virtual = np.array(ugv.desPos[:2])

            ugv_pos_err = ugv_ref_virtual - ugv_virtual_pos
            if np.linalg.norm(ugv_pos_err) < 0.05:
                ugv_pos_err = 0*ugv_pos_err
            ugv_u_nom = ugv.kPos * ugv_pos_err


            u_g_ref[2*i:2*i+2] = ugv_u_nom

        return u_d_ref, u_g_ref

    def publish_centralized_cmds(self, u_d_sol, u_g_sol):
        now = rospy.Time.now()

        # --------------------------------------------------
        # Publish drone commands directly
        # --------------------------------------------------
        if u_d_sol is not None:
            u_d_sol = np.asarray(u_d_sol).reshape(-1)


            for i in range(self.no_agents):
                msg = TwistStamped()
                if self.drones[i].landFlag:
                    msg.twist.linear.z = 0.0

                else:
                    cmd = u_d_sol[3*i:3*i+3]

                    msg.header.stamp = now
                    msg.header.frame_id = "world"

                    msg.twist.linear.x = float(cmd[0])
                    msg.twist.linear.y = float(cmd[1])
                    msg.twist.linear.z = float(cmd[2])

                    msg.twist.angular.x = 0.0
                    msg.twist.angular.y = 0.0
                    msg.twist.angular.z = 0.0


                self.droneCmdPub[i].publish(msg)

        # --------------------------------------------------
        # Publish UGV commands via NID
        # --------------------------------------------------
        if u_g_sol is not None:
            u_g_sol = np.asarray(u_g_sol).reshape(-1)

            for i in range(self.no_agents):
                ugv = self.ugvs[i]
                msg = Twist()
                if ugv.stopFlag:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0

                else:
                    ux, uy = u_g_sol[2*i:2*i+2]

                    v, omega = self.nid_to_unicycle(
                        ux=float(ux),
                        uy=float(uy),
                        theta=float(ugv.yaw),
                        l=float(ugv.off)
                    )

                    msg.linear.x = float(v)
                    msg.linear.y = 0.0
                    msg.linear.z = 0.0

                    msg.angular.x = 0.0
                    msg.angular.y = 0.0
                    msg.angular.z = float(omega)

                self.ugvCmdPub[i].publish(msg)


    def nid_to_unicycle(self, ux, uy, theta, l):
        """
        Map desired virtual-point velocity [ux, uy]
        to unicycle inputs [v, omega].
        """
        v = np.cos(theta) * ux + np.sin(theta) * uy
        omega = (-np.sin(theta) * ux + np.cos(theta) * uy) / l

        v = np.clip(v, -0.1, 0.1)
        omega = np.clip(omega, -0.3, 0.3)
        return v, omega

    def publish_modes(self):
        for i in range(self.no_agents):
            modeMsg = Int8()
            modeMsg.data = self.drones[i].droneMode

            self.droneModePub[i].publish(modeMsg)

            if self.drones[i].landFlag:
                landMsg = Int8()
                landMsg.data = 1
                self.landSignal[i].publish(landMsg)

            modeMsg.data = self.ugvs[i].ugvMode
            self.ugvModePub[i].publish(modeMsg)


    def check_experiment_end(self):
        return all(drone.secondTaskDone and drone.landFlag for drone in self.drones)

    def save_qp_times(self):
        timestamp = time.strftime("%Y%m%d_%H%M%S")

        data = {
            "drone_qp_times": np.array(self.drone_qp_times, dtype=float),
            "ugv_qp_times": np.array(self.ugv_qp_times, dtype=float)
        }


        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("cbf_constraints")

        filename = os.path.join(pkg_path, "time_results", f"qp_times_{timestamp}.npy")
        
        np.save(filename, data)

        rospy.loginfo("Saved QP times to %s", filename)

    def save_h_func(self):
        timestamp = time.strftime("%Y%m%d_%H%M%S")

        data = {
            "drone_h_logs": self.drone_h_logs,
            "ugv_h_logs": self.ugv_h_logs
        }


        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("cbf_constraints")


        filename = os.path.join(pkg_path, "time_results", f"h_func_{timestamp}.npy")
        
        np.save(filename, data)

        rospy.loginfo("Saved H func to %s", filename)


    def loop(self):
        self.publish_modes()
        u_d_sol, u_g_sol, u_d_ref, u_g_ref, A_d, b_d, A_g, b_g = self.solve_centralized_qps()

        if u_d_sol is None:
            rospy.logwarn_throttle(1.0, "Drone centralized QP failed.")
        if u_g_sol is None:
            rospy.logwarn_throttle(1.0, "UGV centralized QP failed.")

        if u_d_sol is not None or u_g_sol is not None:
            self.publish_centralized_cmds(u_d_sol, u_g_sol)




if __name__ == '__main__':
     try:
        rospy.init_node('constraint_updater', anonymous=True)
        no_agents = int(rospy.get_param('~no_of_agents'))

        dc = ConstraintUpdater('constraint_updater', no_agents)
     except rospy.ROSInterruptException:
        pass
