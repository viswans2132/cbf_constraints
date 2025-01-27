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
import cvxpy as cp
import sys
from cbf_constraints.drone_parameters import DroneParameters
from cbf_constraints.ugv_parameters import UgvParameters


class TaskAssigner():	
    def __init__(self, name, no_agents):
        self.name = name
        self.filterFlag = False
        self.no_agents = no_agents

        biggest_sq_no = 2

        while biggest_sq_no*biggest_sq_no < 2*self.no_agents:
        	biggest_sq_no = biggest_sq_no + 1

        self.t = rospy.get_time()
        self.drones = []
        self.ugvs = []

        for i in range(no_agents):
            drone_name = 'dcf' + str(i+1)
            ugv_name = 'demo_turtle' + str(i+1)
            self.drones.append(DroneParameters(drone_name))
            self.ugvs.append(UgvParameters(ugv_name))



        for drone in self.drones:
            self.droneOdomSub.append(rospy.Subscriber('/{}/odometry_sensor1/odometry'.format(drone.name), Odometry, drone.odom_cb))
            self.droneParamSub.append(rospy.Subscriber('/{}/update_params'.format(drone.name), DroneParamsMsg, drone.params_cb))
            self.droneLandSub.append(rospy.Subscriber('/{}/land_signal'.format(drone.name), Int8, drone.land_cb))
            self.droneModeSub.append(rospy.Subscriber('/{}/uav_mode'.format(drone.name), Int8, drone.mode_cb))
            self.droneRefPub.append(rospy.Publisher('/{}/ref'.format(drone.name), DronePosVelMsg, queue_size=10))
            self.droneConsPub.append(rospy.Publisher('/{}/cons'.format(drone.name), DroneConstraintMsg, queue_size=10))
            self.droneModePub.append(rospy.Publisher('/{}/update_uav_mode'.format(drone.name), Int8, queue_size=10))


        for ugv in self.ugvs:
            # self.ugvOdomSub.append(rospy.Subscriber('/vicon/{}/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
            self.ugvOdomSub.append(rospy.Subscriber('/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
            self.ugvParamSub.append(rospy.Subscriber('/{}/update_params'.format(ugv.name), UgvParamsMsg, ugv.params_cb))
            self.ugvModeSub.append(rospy.Subscriber('/{}/ugv_mode'.format(ugv.name), Int8, ugv.mode_cb))
            self.ugvRefPub.append(rospy.Publisher('/{}/ref'. format(ugv.name), UgvPosVelMsg, queue_size=10))
            self.ugvConsPub.append(rospy.Publisher('/{}/cons'. format(ugv.name), UgvConstraintMsg, queue_size=10))
            self.ugvModePub.append(rospy.Publisher('/{}/update_ugv_mode'. format(ugv.name), Int8, queue_size=10))

        maxNorthBound = 1.5
        minSouthBound = -1.5
        minEastBound = -1.5
        maxWestBound = 1.5


        for ugv in self.ugvs:
        	while not ugv.odomFlag:
        		self.rate.sleep()
    		ugv.northBound = ugv.pos[0] + 1.5
    		ugv.southBound = ugv.pos[0] - 1.5
    		ugv.eastBound = ugv.pos[1] - 1.5
    		ugv.westBound = ugv.pos[1] + 1.5

    		maxNorthBound = np.maximum(maxNorthBound, ugv.northBound)
    		minSouthBound = np.minimum(minSouthBound, ugv.southBound)
    		minEastBound = np.minimum(minEastBound, ugv.eastBound)
    		maxWestBound = np.maximum(maxWestBound, ugv.westBound)

		for drone in self.drones:
			drone.northBound = maxNorthBound
			drone.southBound = minSouthBound
			drone.eastBound = minEastBound
			drone.westBound = maxWestBound
