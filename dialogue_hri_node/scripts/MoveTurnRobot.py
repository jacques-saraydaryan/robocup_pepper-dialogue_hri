#!/usr/bin/env python  

import qi
import argparse
import sys
import time
from threading import Timer
import rospy
from collections import namedtuple
import json
import actionlib
from actionlib_msgs.msg import GoalID
from robocup_msgs.msg import gm_bus_msg
from dialogue_hri_srvs.srv import MoveTurn, MoveArmHand,PointAt
from std_msgs.msg import Empty


######### Command to Test
## 
## 
#########


class MoveTurnRobot:
    _session=None
    _memory=None

    def __init__(self,ip,port):
        self._ip=ip
        self._port=port
        while not self.configureNaoqi() and not rospy.is_shutdown():
            rospy.sleep(0.5)
        self.configure()

        rospy.loginfo("MoveHri: READY TO PROCESS ACTION")


    def configureNaoqi(self):
        self._session = qi.Session()
        rospy.loginfo("MoveHRI: try connecting to robot...")
        try:
            self._session.connect("tcp://" + self._ip + ":" + str(self._port))
        except RuntimeError:
            rospy.logerr("Can't connect to Naoqi at ip \"" + self._ip + "\" on port " + str(self._port) +".\n"
                   "Please check your script arguments. Run with -h option for help.")
            return False
        self._motion = self._session.service("ALMotion")
        self._tracker_service = self._session.service("ALTracker")

        rospy.loginfo("MOVE HRI: CONFIGURATION NAOQI OK")
        return True

    def configure(self):
        #create Service
        self._activateMoveSound_service = rospy.Service('move_turn_service', MoveTurn, self.moveturn)
        self._move_arm_service = rospy.Service('move_arm_hand', MoveArmHand, self.moveArmHand)
        self._point_at_service = rospy.Service('point_at', PointAt, self.pointAt)

        rospy.loginfo("MoveHri: CONFIGURATION ACTION OK")

    def moveturn(self,req):
        self._motion.moveTo(0,0,req.turn_rad)
        rospy.loginfo("MoveHri: ROTATION ASKED rad:"+str(req.turn_rad))
        return []

    def moveArmHand(self,req):
        # go to an init head pose.

        #RShoulderPitch LShoulderPitch
        #names = ["RShoulderPitch", "LShoulderPitch"]

        if req.arm_l_or_r == "r":
            names = ["RShoulderPitch"]
        else:
            names = ["LShoulderPitch"]

        angles = [req.turn_rad]
        times = [1.0]
        isAbsolute = True
        self._motion.angleInterpolation(names, angles, times, isAbsolute)

        stiffness = req.stiffness
        timeLists = 1.0
        #self._motion.stiffnessInterpolation(names[0], stiffness, timeLists)
        self._motion.setStiffnesses(names, stiffness)

        # if req.arm_l_or_r == "r":
        #     name = ["RShoulderPitch"]
        # else:
        #     name = ["LShoulderPitch"]
        # angles = req.turn_rad
        # fractionMaxSpeed = .1
        # self._motion.setAngles(name, angles, fractionMaxSpeed)

        rospy.loginfo("MoveHri: Move Arm ASKED rad:" + str(req.turn_rad)+", r or l :"+str(req.arm_l_or_r)+ "stiffness: "+str(stiffness))
        return []

    def pointAt(self, req):
        self._tracker_service.lookAt([req.x, req.y, req.z],0.7,False)

        if req.y > 0:
            name = "LArm"
        else:
            name = "RArm"
        self._tracker_service.pointAt(name, [req.x, req.y, req.z],0, 0.7)

        #FIXME need to return to an idle position after a defined time
        #CAUTION if the condition above is not completed, robot arm still pointing after AND
        # 2 arm pointing at a time could appear
        return []


if __name__ == "__main__":
    rospy.init_node('pepper_move_hri')
    ip=rospy.get_param('~ip',"192.168.1.201")
    port=rospy.get_param('~port',9559)
   
    MoveTurnRobot(ip,port)
    rospy.spin()

