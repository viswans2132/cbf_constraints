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
import cvxpy
import sys
from cbf_constraints.drone_parameters import DroneParameters
from cbf_constraints.ugv_parameters import UgvParameters

def dist(x_):
    return np.linalg.norm(x_)

def sq_dist(x_, y_):
    return np.sum(np.square(x_) / np.square(y_))


class CentralController:
    droneOdomSub = []
    droneParamSub = []
    droneRefPub = []
    droneConsPub = []
    droneModePub = []
    ugvOdomSub = []
    ugvParamSub = []
    ugvRefPub = []
    ugvConsPub = []
    ugvModePub = []
    def __init__(self, name):
        self.name = name
        self.filterFlag = False

        self.t = rospy.get_time()

        self.drones = [DroneParameters('dcf1'), DroneParameters('dcf2'), DroneParameters('dcf3')]
        self.ugvs = [UgvParameters('demo_turtle1'), UgvParameters('demo_turtle2'), UgvParameters('demo_turtle3')]
        self.lenDrones = len(self.drones)
        self.lenUgvs = len(self.ugvs)
        self.rate = rospy.Rate(60)
        self.droneModeSub = rospy.Subscriber('/uav_modes', DroneInt8Array, self.setDroneMode)
        self.ugvModeSub = rospy.Subscriber('/ugv_modes', UgvInt8Array, self.setUgvMode)

        self.offsetW = [0.0, 0.0]

        for drone in self.drones:
            self.droneOdomSub.append(rospy.Subscriber('/{}/odometry_sensor1/odometry'.format(drone.name), Odometry, drone.odom_cb))
            self.droneParamSub.append(rospy.Subscriber('/{}/params'.format(drone.name), DroneParamsMsg, drone.params_cb))
            self.droneRefPub.append(rospy.Publisher('/{}/ref'.format(drone.name), DronePosVelMsg, queue_size=10))
            self.droneConsPub.append(rospy.Publisher('/{}/cons'.format(drone.name), DroneConstraintMsg, queue_size=10))
            self.droneModePub.append(rospy.Publisher('/{}/uav_mode'.format(drone.name), Int8, queue_size=10))


        for ugv in self.ugvs:
            self.ugvOdomSub.append(rospy.Subscriber('/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
            self.ugvParamSub.append(rospy.Subscriber('/{}/params'.format(ugv.name), UgvParamsMsg, ugv.params_cb))
            self.ugvRefPub.append(rospy.Publisher('/{}/ref'. format(ugv.name), UgvPosVelMsg, queue_size=10))
            self.ugvConsPub.append(rospy.Publisher('/{}/cons'. format(ugv.name), UgvConstraintMsg, queue_size=10))
            self.ugvModePub.append(rospy.Publisher('/{}/ugv_mode'. format(ugv.name), Int8, queue_size=10))


        print('Sleeping')
        time.sleep(1)

        for i in range(self.lenDrones):
            modeMsg = Int8()
            modeMsg.data = 1
            self.droneModePub[i].publish(modeMsg)
            self.drones[i].offsetAngle = np.arctan2(self.drones[i].pos[1] - self.offsetW[1], self.drones[i].pos[0] - self.offsetW[0])
            print(self.drones[i].offsetAngle*180/np.pi)

        time.sleep(2)


        while not rospy.is_shutdown():
            self.loop()


    def loop(self):
        # A_ = [np.zeros((1,3))]*self.lenDrones
        # b_ = [0]*self.lenDrones

        A_ = [np.zeros((5,3))]*self.lenUgvs
        b_ = [np.zeros((5,1))]*self.lenUgvs
        C_ = [np.zeros((4,2))]*self.lenUgvs
        d_ = [np.zeros((4,1))]*self.lenUgvs
        i = 0
        self.rate.sleep()
        for droneI in self.drones:
            ugvI = self.ugvs[i]            
            if ugvI.odomFlag:
                C_[i][0,0] = -1
                C_[i][0,1] =  0
                d_[i][0] =  -ugvI.omegaB*(1.5 + self.offsetW[0] - ugvI.off - ugvI.posOff[0])
                C_[i][1,0] = 1
                C_[i][1,1] =  0
                d_[i][1] =  -ugvI.omegaB*(-ugvI.off + ugvI.posOff[0] + 1.5 - self.offsetW[0])
                C_[i][2,0] = 0
                C_[i][2,1] =  -1
                d_[i][2] =  -ugvI.omegaB*(1.5 + self.offsetW[1] - ugvI.off - ugvI.posOff[1])
                # print('h1: {}'.format(ugvI.pos[1] - ugvI.posOff[1]))
                C_[i][3,0] = 0
                C_[i][3,1] =  1
                d_[i][3] =  -ugvI.omegaB*(-ugvI.off + ugvI.posOff[1] + 1.5 - self.offsetW[1])
                # print('h2: {}'.format(ugvI.off + ugvI.posOff[1] + 1.5))
                k = i+1
                # print([i,k])
                for ugvK in self.ugvs[k:]:
                    if ugvK.odomFlag:
                        ugvErrPos = ugvI.posOff - ugvK.posOff
                        if dist(ugvErrPos) < 1.0:
                            sqHorDist = sq_dist(ugvI.pos[:2] - ugvK.pos[:2], np.array([1,1]))
                            rad = ugvI.kRad + 2*ugvI.off
                            h = sqHorDist - rad*rad
                            C_[i] = np.vstack((C_[i], np.array([2*ugvErrPos[0], 2*ugvErrPos[1]])))
                            C_[k] = np.vstack((C_[k], -np.array([2*ugvErrPos[0], 2*ugvErrPos[1]])))
                            # whdhdt = -ugvK.omegaA*h + 2*ugvK.kScaleA*(ugvErrPos[0]*ugvK.vel[0] + ugvErrPos[1]*ugvK.vel[1])
                            # print(whdhdt)
                            d_[i] = np.vstack((d_[i], -ugvK.omegaC*h + 2*(ugvErrPos[0]*ugvK.vel[0] + ugvErrPos[1]*ugvK.vel[1])))
                            d_[k] = np.vstack((d_[k], -ugvK.omegaC*h - 2*(ugvErrPos[0]*ugvI.vel[0] + ugvErrPos[1]*ugvI.vel[1])))
                            # print([C_[i].shape, C_[k].shape, ugvI.name, ugvK.name])
                    k = k + 1
                ugvConsMsg = UgvConstraintMsg()
                try:
                    Cd_ = np.hstack((C_[i], d_[i].reshape((-1,1)))).flatten()
                    ugvConsMsg.constraints = Cd_.tolist()
                    self.ugvConsPub[i].publish(ugvConsMsg)
                except ValueError:
                    # print([A_[i].shape, b_[i].shape, ugvI.name])
                    print('Incompatible C and d')

                refMsg = UgvPosVelMsg()
                refMsg.position = [ugvI.pos[0], ugvI.pos[1]]
                refMsg.velocity = [0.0, 0.0]

                if ugvI.followFlag:
                    # print(np.linalg.norm(ugvI.desPos[:2] - ugvI.pos[:2]))
                    # print(ugvI.reverseFlag)
                    if np.linalg.norm(ugvI.desPos[:2] - ugvI.pos[:2]) < 0.15:
                        if ugvI.reverseFlag:
                            ugvI.reverseFlag = False
                        else:
                            ugvI.reverseFlag = True

                    self.getUgvPosVelMsg(refMsg, i, ugvI.offsetAngle, rospy.get_time())
                    ugvI.desPos = np.array(refMsg.position)

                elif ugvI.returnFlag:
                    if ugvI.odomFlag:
                        refMsg.position = [ugvI.pos[0], ugvI.pos[1], ugvI.pos[2]]
                        refMsg.velocity = [ugvI.vel[0], ugvI.vel[1], ugvI.vel[2]]

                # self.ugvRefPub[i].publish(refMsg)
                # print('Publishing: {}, {}'.format(ugvI.name, refMsg.position))

            if droneI.odomFlag:
                if self.filterFlag:
                    sqHorDist = sq_dist(ugvI.pos[:2] - droneI.pos[:2], np.ones((2,)))
                    ugvErrPos = droneI.pos - ugvI.pos
                    ugvErrVel = droneI.vel[:2] - ugvI.vel[:2]
                    # print("{}: {:.3f}, {:.3f}, {:.3f}".format(ugvI.name, ugvI.pos[0], ugvI.pos[1], ugvI.pos[2]))
                    # print("{:.3f}, {:.3f}, {:.3f}".format(droneI.pos[0], droneI.pos[1], droneI.pos[2]))
                    if sqHorDist < 0.000225 and ugvErrPos[2] < 0.04 and np.linalg.norm(ugvErrVel) < 0.1:
                        # print('Time to initiate landing')
                        # print("{:.3f}, {:.3f}, {:.3f}, {:.3f}".format(ugvErrPos[0], ugvErrPos[1], ugvErrPos[2]))
                        droneModeMsg = Int8()
                        droneModeMsg.data = 2
                        self.droneModePub[i].publish(droneModeMsg)

                    else:
                        A_[i][0,0] = -1
                        A_[i][0,1] =  0
                        b_[i][0] =  -droneI.omegaB*(1.4 + self.offsetW[0] - droneI.pos[0])
                        A_[i][1,0] = 1
                        A_[i][1,1] =  0
                        b_[i][1] =  -droneI.omegaB*(droneI.pos[0] + 1.4 - self.offsetW[0])
                        A_[i][2,0] = 0
                        A_[i][2,1] =  -1
                        b_[i][2] =  -droneI.omegaB*(1.4 + self.offsetW[1] - droneI.pos[1])
                        # print('h1: {}'.format(ugvI.pos[1] - ugvI.posOff[1]))
                        A_[i][3,0] = 0
                        A_[i][3,1] =  1
                        b_[i][3] =  -droneI.omegaB*(droneI.pos[1] + 1.4 - self.offsetW[1])
                        A_[i][4,0] = 0
                        A_[i][4,1] =  -1
                        b_[i][4] =  -droneI.omegaB*(2.0 - droneI.pos[2])

                        h = ugvErrPos[2] - ugvI.kRate*ugvI.kScaleD*sqHorDist*(np.exp(-ugvI.kRate*sqHorDist)) - ugvI.kOffset
                        dhdx = 2*ugvI.kRate*ugvI.kScaleD*(ugvI.kRate*sqHorDist - 1)*np.exp(-ugvI.kRate*sqHorDist)*ugvErrPos[0]
                        dhdy = 2*ugvI.kRate*ugvI.kScaleD*(ugvI.kRate*sqHorDist - 1)*np.exp(-ugvI.kRate*sqHorDist)*ugvErrPos[1]
                        dhdz = 1
                        dhdt = 2*ugvI.kRate*ugvI.kScaleD*(1 - ugvI.kRate)*np.exp(-ugvI.kRate*sqHorDist)*(ugvErrPos[0]*ugvI.vel[0] + ugvErrPos[1]*ugvI.vel[1])
                        # if droneI.name == "dcf6":
                        #     print('H: {}: {}'.format(h, ugvI.name))
                        #     print(- ugvI.omegaD*h)

                        A_[i] = np.vstack((A_[i], np.array([dhdx,dhdy, dhdz])))
                        b_[i] = np.vstack((b_[i], - ugvI.omegaD*h - dhdt))


                        # A_[i][0,0] = 2*ugvI.kRate*ugvI.kScaleD*(ugvI.kRate*sqHorDist - 1)*np.exp(-ugvI.kRate*sqHorDist)*ugvErrPos[0]
                        # A_[i][0,1] = 2*ugvI.kRate*ugvI.kScaleD*(ugvI.kRate*sqHorDist - 1)*np.exp(-ugvI.kRate*sqHorDist)*ugvErrPos[1]
                        # A_[i][0,2] = 1
                        
                        # try:
                        #     b_[i][0] = - ugvI.omegaD*h - 2*ugvI.kRate*ugvI.kScaleD*(1 - ugvI.kRate)*np.exp(-ugvI.kRate*sqHorDist)*(ugvErrPos[0]*ugvI.vel[0] + ugvErrPos[1]*ugvI.vel[1])
                        # except TypeError:
                        #     b_[i] = - ugvI.omegaD*h - 2*ugvI.kRate*ugvI.kScaleD*(1 - ugvI.kRate)*np.exp(-ugvI.kRate*sqHorDist)*(ugvErrPos[0]*ugvI.vel[0] + ugvErrPos[1]*ugvI.vel[1])
                        
                        j = i+1

                        for droneJ in self.drones[j:]:
                            droErrPos = droneI.pos - droneJ.pos
                            if dist(droErrPos) < 1.0:
                                h = sq_dist(droErrPos, np.array([1,1,1])) - droneI.kRad[0]*droneI.kRad[0]
                                scaled_disp = 2*droErrPos
                                # print('h: {:.3f}, {:.3f}, {:.3f}, {:.3f}'.format(h, droErrPos[0], droErrPos[1], droErrPos[2]))
                                # print('scaled_disp: {:.3f}, {:.3f}, {:.3f}'.format(scaled_disp[0], scaled_disp[1], scaled_disp[2]))
                                # print('dhdt: {}'.format(-droneI.omegaC*h + scaled_disp@droneJ.vel))
                                A_[i] = np.vstack((A_[i], scaled_disp))
                                A_[j] = np.vstack((A_[j], -scaled_disp))
                                b_[i] = np.vstack((b_[i], -droneI.omegaC*h + scaled_disp@droneJ.vel))
                                b_[j] = np.vstack((b_[j], -droneJ.omegaC*h - scaled_disp@droneI.vel))
                                # print([b_[j].shape, b_[i].shape])
                                # print(b_)
                            j = j+1

                        for ugvK in self.ugvs:
                            if ugvK != ugvI:
                                ugvErrPos = droneI.pos - ugvK.pos
                                if dist(ugvErrPos) < 1.0:
                                    sqDist = sq_dist(droneI.pos - ugvK.pos, np.array([1,1,1]))
                                    h = sqDist - ugvK.kRad*ugvK.kRad
                                    # if droneI.name == "dcf5":
                                    #     print('h: {:.3f}, {}'.format(h, ugvK.kRad))
                                    A_[i] = np.vstack((A_[i], 2*ugvErrPos))
                                    # whdhdt = -ugvK.omegaA*h + 2*ugvK.kScaleA*(ugvErrPos[0]*ugvK.vel[0] + ugvErrPos[1]*ugvK.vel[1])
                                    # print(whdhdt)
                                    b_[i] = np.vstack((b_[i], -ugvK.omegaA*h + 2*(ugvErrPos[0]*ugvK.vel[0] + ugvErrPos[1]*ugvK.vel[1])))
                        droneConsMsg = DroneConstraintMsg()
                        # print('A_: {}'.format(A_[i]))
                        # print('b_: {}'.format(b_[i]))
                        # # print(np.append(A_[i].flatten(),b_[i]))
                        # print(A_[i].shape)
                        # print('{}: {}'.format(b_[i].shape, droneI.name))
                        try:
                            Ab_ = np.hstack((A_[i], b_[i].reshape((-1,1)))).flatten()
                            # print(Ab_)
                            # droneConsMsg.constraints = np.append(A_[i].flatten(), b_[i]).tolist()
                            droneConsMsg.constraints = Ab_.tolist()
                            self.droneConsPub[i].publish(droneConsMsg)
                        except ValueError:
                            # print([A_[i].shape, b_[i].shape, droneI.name])
                            print('Incompatible A and b')

                        refMsg = DronePosVelMsg()
                        refMsg.position = [droneI.pos[0], droneI.pos[1], 0.5]
                        refMsg.velocity = [0.0, 0.0, 0.0]
                        refMsg.yaw = 0.0
                        refMsg.yawVelocity = 0.0

                        if droneI.followFlag:
                            self.getDronePosVelMsg(refMsg, droneI.offsetAngle, rospy.get_time())

                        elif droneI.returnFlag:
                            if ugvI.odomFlag:
                                refMsg.position = [ugvI.pos[0], ugvI.pos[1], ugvI.pos[2]]
                                refMsg.velocity = [ugvI.vel[0], ugvI.vel[1], ugvI.vel[2]]
                            # else:
                            #     # print("UGV position not received. Landing at the current position")
                            #     refMsg.position = [droneI.pos[0], droneI.pos[1], 0.5]
                            #     droneModeMsg = Int8()
                            #     droneModeMsg.data = 2
                            #     self.droneModePub[i].publish(droneModeMsg)


                        self.droneRefPub[i].publish(refMsg)
                        # print('Publishing: {}'.format(droneI.name))
            else:
                # print('Odometry Not received for {}'.format(droneI.name))
                pass
            i = i + 1
        # self.rate.sleep()


    def setDroneMode(self, msg):
        if len(msg.data) != len(self.droneModePub):
            print("Invalid number of values in the msg. Currently, there are {} drones active.".format(self.lenDrones))
        else:
            if self.filterFlag == False:                
                for i in range(self.lenUgvs):
                    modeMsg = Int8()
                    modeMsg.data = 1
                    self.ugvModePub[i].publish(modeMsg)
                    self.ugvs[i].offsetAngle = np.arctan2(self.ugvs[i].pos[1] - self.offsetW[1], self.ugvs[i].pos[0] - self.offsetW[0])

                print('Filter Active')
                self.filterFlag = True
                self.t = rospy.get_time()
            for i in range(self.lenDrones):
                droneModeMsg = Int8()
                if msg.data[i] == 0 or msg.data[i] == 2:
                    droneModeMsg.data = msg.data[i]
                    self.droneModePub[i].publish(droneModeMsg)
                    self.drones[i].followFlag = True
                    self.drones[i].returnFlag = False

                elif msg.data[i] == 1:
                    droneModeMsg.data = 0
                    self.droneModePub[i].publish(droneModeMsg)
                    self.drones[i].returnFlag = True
                    self.drones[i].followFlag = False
                    print('Return Flag On')

                else:
                    print("Invalid mode for drone {}. Available modes: 0 - Follow Trajectory, 1 - Return and Land 2 - Safety Land".format(self.drones[i].name))
    
    def setUgvMode(self, msg):
        if len(msg.data) != len(self.ugvModePub):
            print("Invalid number of values in the msg. Currently, there are {} ugvs active.".format(self.lenUgvs))
        else:
            for i in range(self.lenUgvs):
                ugvModeMsg = Int8()
                ugvModeMsg.data = msg.data[i]
                self.ugvModePub[i].publish(ugvModeMsg)



    def land_cb(self, data):
        self.landFlag = True
        print('Safety Landing: Active')

    def getDronePosVelMsg(self, msg, offset, time):
        time =  time - self.t
        msg.position = [np.cos(offset + time/5) + self.offsetW[0], np.sin(offset + time/5) + self.offsetW[1], 0.6]
        # msg.position = [0.31, -0.7, 0.6]
        msg.velocity = [-np.sin(offset + time/5)/5, np.cos(offset + time/5)/5, 0.0]
        # msg.velocity = [0.0, -0.0, 0.0]
        msg.yaw = 0.0
        msg.yawVelocity = 0.0


    def getUgvPosVelMsg(self, msg, i, offset, time):
        time =  time - self.t
        if i == 1:
            msg.position = [np.cos(offset - time/10) + self.offsetW[0], np.sin(offset - time/10) + self.offsetW[1]]
            msg.velocity = [np.sin(offset - time/10)/10, -np.cos(offset - time/10)/10]
        elif i == 2:
            msg.position = [-1.5*np.cos(offset + time/12) + self.offsetW[0], -0.3*np.sin(offset + time/12) + self.offsetW[1]]
            msg.velocity = [1.5*np.sin(offset + time/12)/12, -0.3*np.cos(offset + time/12)/12]
            # # if (int(time/30))%2 == 0:
            # if flag:    
            #     msg.position = [1.3 + self.offsetW[0], 0.0 + self.offsetW[1]]
            #     msg.velocity = [0.0, 0.0]
            # else:
            #     msg.position = [-1.3 + self.offsetW[0], 0.0 + self.offsetW[1]]
            #     msg.velocity = [0.0, 0.0] 
        else:
            msg.position = [0.3*np.sin(offset + time/10) + self.offsetW[0], 1.5*np.cos(offset + time/10) + self.offsetW[1]]
            msg.velocity = [0.3*np.cos(offset + time/10)/10, -1.5*np.sin(offset + time/10)/10]
            # if flag:
            # # if (int(time/30))%2 == 0:
            #     msg.position = [0.0 + self.offsetW[0], 1.3 + self.offsetW[1]]
            #     msg.velocity = [0.0, 0.0]
            # else:
            #     msg.position = [0.0 + self.offsetW[0], -1.3 + self.offsetW[1]]
            #     msg.velocity = [0.0, 0.0] 







    def obs_cb(self, data):
        self.obs_pos[0] = float(data.pose.pose.position.x)
        self.obs_pos[1] = float(data.pose.pose.position.y)
        self.obs_pos[2] = float(data.pose.pose.position.z)
        self.obs_pos[3] = float(data.twist.twist.linear.x)
        self.obs_pos[4] = float(data.twist.twist.linear.y)
        self.obs_pos[5] = float(data.twist.twist.linear.z)


    def ref_cb(self, data):
        if(self.follow_flag):
            self.pos_off[0] = float(data.pose.pose.position.x)
            self.pos_off[1] = float(data.pose.pose.position.y)
            self.pos_off[2] = float(data.pose.pose.position.z)



if __name__ == '__main__':
     try:
        rospy.init_node('crazyflie_controller', anonymous=True)
        dc = CentralController('controller')
     except rospy.ROSInterruptException:
        pass
