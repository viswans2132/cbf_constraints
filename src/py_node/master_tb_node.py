 #!/usr/bin/env python
 # license removed for brevity
import rospy
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header, Int8
from tb_cbf.msg import UgvPosVelMsg, UgvConstraintMsg, UgvParamsMsg, UgvInt8Array
import time
import numpy as np
import cvxpy
import sys
from cbf_constraints.ugv_parameters import UgvParameters

def dist(x_):
    return np.linalg.norm(x_)

def sq_dist(x_, y_):
    return np.sum(np.square(x_) / np.square(y_))


class UgvController:
    ugvOdomSub = []
    ugvParamSub = []
    ugvRefPub = []
    ugvConsPub = []
    ugvModePub = []
    ugvCmdPub = []
    def __init__(self, name):
        self.name = name
        self.filterFlag = False

        self.t = rospy.get_time()

        self.ugvs = [UgvParameters('demo_turtle1'), UgvParameters('demo_turtle2'), UgvParameters('demo_turtle3'), UgvParameters('demo_turtle4')]
        self.lenUgvs = len(self.ugvs)
        self.rate = rospy.Rate(60)
        self.modeSub = rospy.Subscriber('/ugv_modes', UgvInt8Array, self.setMode)

        self.offsetW = [-0.5, -0.5]



        for ugv in self.ugvs:
            # self.ugvOdomSub.append(rospy.Subscriber('/vicon/{}/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
            self.ugvOdomSub.append(rospy.Subscriber('/{}/odom'.format(ugv.name), Odometry, ugv.odom_cb))
            self.ugvParamSub.append(rospy.Subscriber('/{}/params'.format(ugv.name), UgvParamsMsg, ugv.params_cb))
            self.ugvRefPub.append(rospy.Publisher('/{}/ref'.format(ugv.name), UgvPosVelMsg, queue_size=10))
            self.ugvConsPub.append(rospy.Publisher('/{}/cons'.format(ugv.name), UgvConstraintMsg, queue_size=10))
            self.ugvModePub.append(rospy.Publisher('/{}/uav_mode'.format(ugv.name), Int8, queue_size=10))


        print('Sleeping')
        time.sleep(1)
        print('Awake')

        for i in range(self.lenUgvs):
            modeMsg = Int8()
            modeMsg.data = 1
            self.ugvModePub[i].publish(modeMsg)

        while not rospy.is_shutdown():
            self.loop()


    def loop(self):
        C_ = [np.zeros((4,2))]*self.lenUgvs
        d_ = [np.zeros((4,1))]*self.lenUgvs
        i = 0
        for ugvI in self.ugvs:
            j = i+1
            # print(ugvI.odomFlag)
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
                for ugvK in self.ugvs[j:]:
                    if ugvK.odomFlag:
                        ugvErrPos = ugvI.posOff - ugvK.posOff
                        if dist(ugvErrPos) < 2.0:
                            sqHorDist = sq_dist(ugvI.pos[:2] - ugvK.pos[:2], np.array([1,1]))
                            rad = ugvI.kRad + 2*ugvI.off
                            h = sqHorDist - rad*rad
                            C_[i] = np.vstack((C_[i], np.array([2*ugvErrPos[0], 2*ugvErrPos[1]])))
                            C_[j] = np.vstack((C_[j], -np.array([2*ugvErrPos[0], 2*ugvErrPos[1]])))
                            # whdhdt = -ugvK.omegaA*h + 2*ugvK.kScaleA*(ugvErrPos[0]*ugvK.vel[0] + ugvErrPos[1]*ugvK.vel[1])
                            # print(whdhdt)
                            d_[i] = np.vstack((d_[i], -ugvK.omegaC*h + 2*(ugvErrPos[0]*ugvK.vel[0] + ugvErrPos[1]*ugvK.vel[1])))
                            d_[j] = np.vstack((d_[j], -ugvK.omegaC*h - 2*(ugvErrPos[0]*ugvI.vel[0] + ugvErrPos[1]*ugvI.vel[1])))
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
                    self.getPosVelMsg(refMsg, i, rospy.get_time())

                elif ugvI.returnFlag:
                    if ugvI.odomFlag:
                        refMsg.position = [ugvI.pos[0], ugvI.pos[1], ugvI.pos[2]]
                        refMsg.velocity = [ugvI.vel[0], ugvI.vel[1], ugvI.vel[2]]

                self.ugvRefPub[i].publish(refMsg)
                # print('Publishing: {}, {}'.format(ugvI.name, refMsg.position))
                self.rate.sleep()
            i = i+1


    def setMode(self, msg):
        if len(msg.data) != len(self.ugvModePub):
            print("Invalid number of values in the msg. Currently, there are {} ugvs active.".format(self.lenUgvs))
        else:
            if self.filterFlag == False:
                print('Filter Active')
                self.filterFlag = True
                self.t = rospy.get_time()
            for i in range(self.lenUgvs):
                ugvModeMsg = Int8()
                if msg.data[i] == 0 or msg.data[i] == 2:
                    ugvModeMsg.data = msg.data[i]
                    self.ugvModePub[i].publish(ugvModeMsg)
                    self.ugvs[i].followFlag = True
                    self.ugvs[i].returnFlag = False

                elif msg.data[i] == 1:
                    ugvModeMsg.data = 0
                    self.ugvModePub[i].publish(ugvModeMsg)
                    self.ugvs[i].returnFlag = True
                    self.ugvs[i].followFlag = False
                    print('Return Flag On')

                else:
                    print("Invalid mode for ugv {}. Available modes: 0 - Follow Trajectory, 1 - Return and Land 2 - Safety Land".format(self.ugvs[i].name))

    def land_cb(self, data):
        self.landFlag = True
        print('Safety Landing: Active')

    def getPosVelMsg(self, msg, i, time):
        time =  time - self.t
        if i == 1:
            msg.position = [np.cos(time/10) + self.offsetW[0], np.sin(time/10) + self.offsetW[1]]
            msg.velocity = [-np.sin(time/10)/10, np.cos(time/10)/10]
        elif i == 2:
            if (int(time/30))%2 == 0:
                msg.position = [1.5 + self.offsetW[0], 0.0 + self.offsetW[1]]
                msg.velocity = [0.0, 0.0]
            else:
                msg.position = [-1.5 + self.offsetW[0], 0.0 + self.offsetW[1]]
                msg.velocity = [0.0, 0.0] 
        else:
            if (int(time/30))%2 == 0:
                msg.position = [0.0 + self.offsetW[0], 1.5 + self.offsetW[1]]
                msg.velocity = [0.0, 0.0]
            else:
                msg.position = [0.0 + self.offsetW[0], -1.5 + self.offsetW[1]]
                msg.velocity = [0.0, 0.0] 






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
        rospy.init_node('turtlebot_controller', anonymous=True)
        dc = UgvController('controller')
     except rospy.ROSInterruptException:
        pass
