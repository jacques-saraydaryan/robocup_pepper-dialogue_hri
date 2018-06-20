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
from dialogue_hri_srvs.srv import MoveTurn


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

        rospy.loginfo("MoveSoundHri: READY TO PROCESS ACTION")


    def configureNaoqi(self):
        self._session = qi.Session()
        rospy.loginfo("MoveSoundHri: try connecting to robot...")
        try:
            self._session.connect("tcp://" + self._ip + ":" + str(self._port))
        except RuntimeError:
            rospy.logerr("Can't connect to Naoqi at ip \"" + self._ip + "\" on port " + str(self._port) +".\n"
                   "Please check your script arguments. Run with -h option for help.")
            return False
        self._motion = self._session.service("ALMotion")


        rospy.loginfo("MOVESOUND: CONFIGURATION NAOQI OK")
        return True

    def configure(self):
        #create Service
        self._activateMoveSound_service = rospy.Service('move_turn_service', MoveTurn, self.moveturn)
        rospy.loginfo("MoveSoundHri: CONFIGURATION ACTION OK")

    def moveturn(self,req):
        self._motion.moveTo(0,0,req.turn_rad)
        return


if __name__ == "__main__":
    rospy.init_node('pepper_move_sound_hri')
    ip=rospy.get_param('~ip',"169.254.17.15")
    port=rospy.get_param('~port',9559)
   
    MoveTurnRobot(ip,port)
    rospy.spin()

