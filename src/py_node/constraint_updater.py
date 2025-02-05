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


class ConstraintUpdater:
    droneLandSub = []
    droneOdomSub = []
    droneParamSub = []
    droneModeSub = []
    droneParamPub = []
    droneConsPub = []
    droneModePub = []
    ugvOdomSub = []
    ugvParamSub = []
    ugvRefPub = []
    ugvModeSub = []
    ugvConsPub = []
    ugvModePub = []
    def __init__(self, name, no_agents):
        self.name = name
        self.filterFlag = False
        self.no_agents = no_agents

        self.t = rospy.get_time()
        self.drones = []
        self.ugvs = []

        for i in range(no_agents):
            drone_name = 'dcf' + str(i+1)
            ugv_name = 'demo_turtle' + str(i+1)
            self.drones.append(DroneParameters(drone_name))
            self.ugvs.append(UgvParameters(ugv_name))

        self.rate = rospy.Rate(60)

        for drone in self.drones:
            self.droneOdomSub.append(rospy.Subscriber('/{}/odometry_sensor1/odometry'.format(drone.name), Odometry, drone.odom_cb))
            self.droneParamSub.append(rospy.Subscriber('/{}/update_params'.format(drone.name), DroneParamsMsg, drone.params_cb))
            self.droneRefSub.append(rospy.Publisher('/{}/ref'.format(drone.name), DronePosVelMsg, queue_size=10))
            self.droneParamPub.append(rospy.Publisher('/{}/params'.format(drone.name), DroneParamsMsg, queue_size=10))
            self.droneConsPub.append(rospy.Publisher('/{}/cons'.format(drone.name), DroneConstraintMsg, queue_size=10))


        for ugv in self.ugvs:
            # self.ugvOdomSub.append(rospy.Subscriber('/vicon/{}/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
            self.ugvOdomSub.append(rospy.Subscriber('/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
            self.ugvParamSub.append(rospy.Subscriber('/{}/update_params'.format(ugv.name), UgvParamsMsg, ugv.params_cb))
            self.ugvRefPub.append(rospy.Publisher('/{}/ref'. format(ugv.name), UgvPosVelMsg, queue_size=10))
            self.ugvConsPub.append(rospy.Publisher('/{}/cons'. format(ugv.name), UgvConstraintMsg, queue_size=10))
            self.ugvParamPub.append(rospy.Publisher('/{}/params'.format(ugv.name), UgvParamsMsg, queue_size=10))


        print('Sleeping')
        time.sleep(1)


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
            while not ugv.odomFlag:
                self.rate.sleep()
            ugv.updateBounds([maxBoundX, minBoundX, maxBoundY, minBoundY])
            rospy.loginfo("Updating the bounds for UGV: %s", ugv.name)

            while not drone.odomFlag:
                self.rate.sleep()

            drone.updateBounds([maxBoundX, minBoundX, maxBoundY, minBoundY, maxBoundZ, minBoundZ])
            rospy.loginfo("Updating the bounds for drone: %s", drone.name)
            
            rospy.loginfo("Checking the params for drone: %s", drone.name)
            while not drone.paramFlag:
                self.rate.sleep()
            
            paramMsg = DroneParamsMsg()
            self.droneParamPub[i].publish(paramMsg)
            rospy.loginfo("Received params for drone: %s", drone.name)
            
            rospy.loginfo("Checking the params for ugv: %s", ugv.name)
            while not ugv.paramFlag:
                self.rate.sleep()

            paramMsg = UgvParamsMsg()
            self.ugvParamPub[i].publish(paramMsg)
            rospy.loginfo("Received params for ugv: %s", ugv.name)


        try:
            while not rospy.is_shutdown():
                self.loop()
                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo('Shutting down the node.')

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
        self.A_[i][4,1] =  -1
        self.b_[i][4] =  -droneI.omegaB*(droneI.maxBoundZ - droneI.pos[2])

    def updateLandCons(self, i):
        droneI = self.drones[i]
        ugvI = self.ugvs[i]
        


    def loop():        
        self.A_ = [np.zeros((5,3))]*self.no_agents
        self.b_ = [np.zeros((5,1))]*self.no_agents
        self.C_ = [np.zeros((4,2))]*self.no_agents
        self.d_ = [np.zeros((4,1))]*self.no_agents

        for i in range(no_agents):
            droneI = self.drones[i]
            ugvI = self.ugvs[i]

            self.updateUgvBoundCons(i)

            for k in range(1, no_agents):
                ugvK  =





if __name__ == '__main__':
     try:
        rospy.init_node('constraint_updater', anonymous=True)
        no_agents = int(rospy.get_param('~no_of_agents'))

        dc = ConstraintUpdater('constraint_updater', no_agents)
     except rospy.ROSInterruptException:
        pass
