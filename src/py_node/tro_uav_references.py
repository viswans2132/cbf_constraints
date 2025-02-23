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


class UavController:
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
        self.ugvs = [UgvParameters('demo_turtle1'), UgvParameters('demo_turtle4'), UgvParameters('demo_turtle3')]
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
        
        self.setpoints = 0.5*np.arange(-3,4)

        for i in range(self.lenDrones):
            modeMsg = Int8()
            modeMsg.data = 1
            self.droneModePub[i].publish(modeMsg)
            self.drones[i].offsetAngle = np.arctan2(self.drones[i].pos[1] - self.offsetW[1], self.drones[i].pos[0] - self.offsetW[0])
            self.drones[i].desPos1 = [self.setpoints[np.random.randint(7)], self.setpoints[np.random.randint(7)], 0.8]
            self.drones[i].desPos2 = [self.setpoints[np.random.randint(7)], self.setpoints[np.random.randint(7)], 0.8]

        time.sleep(2)

        for i in range(self.lenUgvs):
            modeMsg = Int8()
            modeMsg.data = 1
            self.ugvModePub[i].publish(modeMsg)






        while not rospy.is_shutdown():
            self.loop()


    def loop(self):
        i = 0
        for droneI in self.drones:
            ugvI = self.ugvs[i]            
            if ugvI.odomFlag:

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

                    self.getUgvPosVelMsg(refMsg, i, ugvI.reverseFlag, rospy.get_time())
                    ugvI.desPos = np.array(refMsg.position)

                elif ugvI.returnFlag:
                    if ugvI.odomFlag:
                        refMsg.position = [ugvI.pos[0], ugvI.pos[1], ugvI.pos[2]]
                        refMsg.velocity = [ugvI.vel[0], ugvI.vel[1], ugvI.vel[2]]

                # self.ugvRefPub[i].publish(refMsg)
                # print('Publishing: {}, {}'.format(ugvI.name, refMsg.position))

            if droneI.odomFlag:
                if self.filterFlag:

                        refMsg = DronePosVelMsg()
                        refMsg.position = [droneI.pos[0], droneI.pos[1], 0.5]
                        refMsg.velocity = [0.0, 0.0, 0.0]
                        refMsg.yaw = 0.0
                        refMsg.yawVelocity = 0.0

                        if droneI.followFlag:
                            if droneI.firstTask:
                                if np.linalg.norm(self.droneI.pos - self.droneI.desPos1) < 0.1:
                                    droneI.firstTask = False
                                refMsg.position = self.droneI.desPos1.tolist()
                            elif droneI.secondTask:
                                if np.linalg.norm(self.droneI.pos - self.droneI.desPos2) < 0.1:
                                    droneI.secondTask = False
                                refMsg.position = self.droneI.desPos2.tolist()

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
        self.rate.sleep()


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

    def oldGetDronePosVelMsg(self, msg, offset, time):
        time =  time - self.t
        msg.position = [np.cos(offset + time/5), np.sin(offset + time/5), 0.7]
        # msg.position = [0, 0, 0.5]
        msg.velocity = [-np.sin(offset + time/5)/5, np.cos(offset + time/5)/5, 0.0]
        # msg.velocity = [0.0, -0.0, 0.0]
        msg.yaw = 0.0
        msg.yawVelocity = 0.0

    def getDronePosVelMsg(self, msg):
        sample = 0.5*np.arange(-3,4)
        msg.position = [sample[np.random.randint(7)], sample[np.random.randint(7)], 0.8]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.yaw = 0.0
        msg.yawVelocity = 0.0


    def getUgvPosVelMsg(self, msg, i, flag, time):
        time =  time - self.t
        if i == 1:
            msg.position = [-np.cos(time/20) + self.offsetW[0], np.sin(time/20) + self.offsetW[1]]
            msg.velocity = [np.sin(time/20)/20, np.cos(time/20)/20]
        elif i == 2:
            msg.position = [-1.2*np.cos(time/10) + self.offsetW[0], -0.3*np.sin(time/10) + self.offsetW[1]]
            msg.velocity = [1.2*np.sin(time/10)/10, -0.3*np.cos(time/10)/10]
            # # if (int(time/30))%2 == 0:
            # if flag:    
            #     msg.position = [1.3 + self.offsetW[0], 0.0 + self.offsetW[1]]
            #     msg.velocity = [0.0, 0.0]
            # else:
            #     msg.position = [-1.3 + self.offsetW[0], 0.0 + self.offsetW[1]]
            #     msg.velocity = [0.0, 0.0] 
        else:
            msg.position = [0.3*np.sin(time/10) + self.offsetW[0], 1.2*np.cos(time/10) + self.offsetW[1]]
            msg.velocity = [0.3*np.cos(time/10)/10, -1.2*np.sin(time/10)/10]
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
