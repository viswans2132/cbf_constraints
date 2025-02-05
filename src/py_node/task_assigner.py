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
import numpy.linalg as la
import cvxpy as cp
import sys
from cbf_constraints.drone_parameters import DroneParameters
from cbf_constraints.ugv_parameters import UgvParameters


class TaskAssigner():	
    def __init__(self, name, no_agents):
        self.name = name
        self.filterFlag = False
        self.no_agents = no_agents

        sq_no = 2
        while sq_no*sq_no < 2*self.no_agents:
        	sq_no = sq_no + 1

        self.t = rospy.get_time()
        self.drones = []
        self.ugvs = []

        self.offset = [0.0, 0.0]        
        self.rate = rospy.Rate(10)
        self.droneUpdateModeSub = rospy.Subscriber('/uav_modes', DroneInt8Array, self.setDroneMode)
        self.ugvUpdateModeSub = rospy.Subscriber('/ugv_modes', UgvInt8Array, self.setUgvMode)

        for i in range(self.no_agents):
            drone_name = 'dcf' + str(i+1)
            ugv_name = 'demo_turtle' + str(i+1)
            self.drones.append(DroneParameters(drone_name))
            self.ugvs.append(UgvParameters(ugv_name))

        for drone in self.drones:
            self.droneOdomSub.append(rospy.Subscriber('/{}/odometry_sensor1/odometry'.format(drone.name), Odometry, drone.odom_cb))
            self.droneLandSub.append(rospy.Subscriber('/{}/land_signal'.format(drone.name), Int8, drone.land_cb))
            self.droneModeSub.append(rospy.Subscriber('/{}/uav_mode'.format(drone.name), Int8, drone.mode_cb))
            self.droneRefPub.append(rospy.Publisher('/{}/ref'.format(drone.name), DronePosVelMsg, queue_size=10))
            self.droneModePub.append(rospy.Publisher('/{}/update_uav_mode'.format(drone.name), Int8, queue_size=10))


        for ugv in self.ugvs:
            self.ugvOdomSub.append(rospy.Subscriber('/vicon/{}/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
            # self.ugvOdomSub.append(rospy.Subscriber('/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
            self.ugvModeSub.append(rospy.Subscriber('/{}/ugv_mode'.format(ugv.name), Int8, ugv.mode_cb))
            self.ugvRefPub.append(rospy.Publisher('/{}/ref'. format(ugv.name), UgvPosVelMsg, queue_size=10))
            self.ugvModePub.append(rospy.Publisher('/{}/update_ugv_mode'. format(ugv.name), Int8, queue_size=10))

        maxBoundX = sq_no*0.5 + 1.5
        minBoundX = -maxBoundX
        minBoundY = -maxBoundX
        maxBoundY = maxBoundX
        maxBoundZ = 2.0
        minBoundZ = 0.0

        self.droneSetpoints = 0.1*np.arange(0,27) - 1.3

        for i in range(self.no_agents):
            ugv = self.ugvs[i]
            drone = self.drones[i]
        	while not ugv.odomFlag:
        		self.rate.sleep()
            ugv.updateBounds([maxBoundX, minBoundX, maxBoundY, minBoundY])
            self.updateUgvSetpoint(ugv)

            while not drone.odomFlag:
                self.rate.sleep()

            drone.updateBounds([maxBoundX, minBoundX, maxBoundY, minBoundY, maxBoundZ, minBoundZ])

            self.updateDroneSetpoint(self, drone)

            while(drone.droneMode != 1):
                self.publishDroneMode(i,1)
                self.rate.sleep()

            while(ugv.ugvMode !=1):
                self.publishUgvMode(i,1)
                self.rate.sleep()


        try:
            while not rospy.is_shutdown():
                self.loop()
                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo('Shutting down the node.')


    def updateUgvSetpoint(self, index):
        ugv = self.ugvs[index]
        ugv.desPos = np.array([ugv.setpoints[0][np.random.randint(len(ugv.setpoints[0])) + self.offset[0]], 
                                ugv.setpoints[1][np.random.randint(len(ugv.setpoints[1])) + self.offset[1]]])
        ugv.desVel = np.array([0.0, 0.0])

    def updateDroneSetpoint(self, index):
        drone = self.drones[index]
        ugv = self.ugvs[index]        
        if drone.returnFlag:
            drone.desPos = np.array([ugv.pos[0], ugv.pos[1], ugv.pos[2]])
            drone.desVel = np.array([ugv.vel[0], ugv.vel[1], ugv.vel[2]])
        else:
            drone.desPos = np.array([self.droneSetpoints[np.random.randint(len(self.droneSetpoints))] + self.offset[0], 
                                    self.droneSetpoints[np.random.randint(len(self.droneSetpoints))] + self.offset[1], 0.8])
            drone.desVel = np.array([0.0, 0.0, 0.0])


    def publishDroneRef(self, index):
        drone = self.drones[index]
        refMsg = DronePosVelMsg()
        refMsg.position = [drone.desPos[0], drone.desPos[1], drone.desPos[2]]
        refMsg.velocity = [drone.desVel[0], drone.desVel[1], drone.desVel[2]]
        refMsg.yaw = 0.0
        refMsg.yawVelocity = 0.0
        self.droneRefPub[index].publish(refMsg)

    def publishUgvRef(self, index):
        ugv = self.ugvs[index]
        refMsg = UgvPosVelMsg()
        refMsg.position = [ugv.desPos[0], ugv.desPos[1]]
        refMsg.velocity = [0.0, 0.0]
        self.ugvRefPub[index].publish(refMsg)

    def publishDroneMode(self, index, mode):
        droneModeMsg = Int8()
        droneModeMsg.data = mode
        self.droneModePub[index].publish(droneModeMsg)

    def publishUgvMode(self, index, mode):
        ugvModeMsg = Int8()
        ugvModeMsg.data = mode
        self.ugvModePub[index].publish(ugvModeMsg)

        def setDroneMode(self, msg):
        if len(msg.data) != len(self.droneModePub):
            print("Invalid number of values in the msg. Currently, there are {} drones active.".format(self.lenDrones))
        else:
            if self.filterFlag == False:
                for i in range(self.lenUgvs):
                    modeMsg = Int8()
                    modeMsg.data = 1
                    self.ugvModePub[i].publish(modeMsg)
                    # self.ugvs[i].offsetAngle = np.arctan2(self.ugvs[i].pos[1] - self.offsetW[1], self.ugvs[i].pos[0] - self.offsetW[0])

                print('Filter Active')
                self.filterFlag = True
                self.t = rospy.get_time()
            for i in range(self.no_agents):
                droneModeMsg = Int8()
                if msg.data[i] == 0 or msg.data[i] == 2:
                    droneModeMsg.data = msg.data[i]
                    self.droneModePub[i].publish(droneModeMsg)
                    self.drones[i].returnFlag = False

                elif msg.data[i] == 1:
                    droneModeMsg.data = 0
                    self.droneModePub[i].publish(droneModeMsg)
                    self.drones[i].returnFlag = True
                    print('Return Flag On')

                else:
                    print("Invalid mode for drone {}. Available modes: 0 - Follow Trajectory, 1 - Return and Land, and 2 - Safety Land".format(self.drones[i].name))
    
    def setUgvMode(self, msg):
        if len(msg.data) != len(self.ugvModePub):
            print("Invalid number of values in the msg. Currently, there are {} ugvs active.".format(self.lenUgvs))
        else:
            for i in range(self.no_agents):
                ugvModeMsg = Int8()
                ugvModeMsg.data = msg.data[i]
                self.ugvModePub[i].publish(ugvModeMsg)


    def loop(self):
        for i in range(self.no_agents):
            drone = self.drones[no_agents]
            ugv = self.uavs[no_agents]
            if drone.returnFlag:
                self.updateDroneSetpoint(i)
                ugvPosErr = drone.pos - ugv.pos
                ugvVelErr = drone.vel - ugv.vel
                if la.norm(ugvPosErr[:2]) < 0.0009 and ugvPosErr[2] < 0.04 and la.norm(ugvVelErr) < 0.2:
                    if drone.droneMode != 0:
                        self.publishDroneMode(i,2)

            else:
                droneErr = drone.pos - drone.desPos
                if la.norm(droneErr) < 0.1:
                    drone.returnFlag =  True
                    self.rate.sleep()

                if not firstTask:
                    if (rospy.get_time() - drone.landTime) > 5.0:
                        if drone.droneMode != 0:
                            self.publishDroneMode(i,0)


            self.publishDroneRef(i)

            ugvErr = ugv.pos - ugv.desPos
            if la.norm(ugvErr) < 0.1:
                self.updateUgvSetpoint(i)

            self.publishUgvRef(i)


if __name__ == '__main__':
     try:
        rospy.init_node('task_assigner', anonymous=True)
        no_agents = int(rospy.get_param('~no_of_agents'))

        dc = TaskAssigner('task_assigner', no_agents)
     except rospy.ROSInterruptException:
        pass
