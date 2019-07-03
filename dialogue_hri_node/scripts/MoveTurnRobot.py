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
from dialogue_hri_srvs.srv import MoveTurn, MoveArmHand,PointAt,GoToCarryPose,ReleaseArms, TurnToInterestPoint
from std_msgs.msg import Empty
import threading

from tf import TransformListener
from robocup_msgs.msg import InterestPoint
from geometry_msgs.msg import Pose
import tf
from math import atan2, fmod, pi, sqrt, fabs
from copy import deepcopy
from map_manager.srv import getitP_service


######### Command to Test
## rosservice call /point_at "{x: 10.0, y: 10.0, z: 0.0, move_head: true, move_arm: true, pose_duration: 8.0}"
## rosservice call /move_arm_hand "{arm_l_or_r: 'l',turn_rad: 0.0,stiffness: 0.8}"
## rosservice call /go_to_carry_pose "{arm_l_or_r:'l', keep_pose: true, stiffness: 0.8}"
## rosservice call /turn_to_interest_point "{itP_label: 'KITCHEN_BIN_02'}"
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

        self._conf_TurnToInterestPoint()

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
        self._go_to_carry_pose = rospy.Service('go_to_carry_pose', GoToCarryPose, self.gotoCarryPose)
        self._release_arms = rospy.Service('release_arms', ReleaseArms, self.releaseArmSrv)
        self._turn_to_interest_point = rospy.Service('turn_to_interest_point', TurnToInterestPoint, self.turnToInterestPoint)

        rospy.loginfo("MoveHri: CONFIGURATION ACTION OK")

    def moveturn(self,req):
        self._motion.moveTo(0,0,req.turn_rad)
        rospy.loginfo("MoveHri: ROTATION ASKED rad:"+str(req.turn_rad))
        return []

    def moveArmHand(self,req):
        self.release_fix_pose_thread()
        # go to an init head pose.
        if req.arm_l_or_r == "r":
            names = ["RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw","RWristYaw"]
            name_hand="RHand"
            name_stiffness = ["RArm"]
            #angles = [req.turn_rad, -0.09, 0.26, -0.14, 1.74]-0.349066
            angles = [req.turn_rad, -0.349066, 0.26, -0.14, 1.74]
        else:
            names = ["LShoulderPitch","LShoulderRoll","LElbowRoll","LElbowYaw","LWristYaw"]
            name_hand = "LHand"
            name_stiffness = ["LArm"]
            #angles = [req.turn_rad, -0.09, 0.26, 0.035, -1.17]
            angles = [req.turn_rad, 0.349066, 0.26, 0.035, -1.17]

        self._motion.stiffnessInterpolation(name_stiffness, 1.0, 1.0)

        times = [2.0,1.0,2.0,2.0,2.0,2.0]
        isAbsolute = True
        self._motion.angleInterpolation(names, angles, times, isAbsolute)
        self._motion.angleInterpolation(name_hand, 0.83, 1.0, True)

        self._motion.stiffnessInterpolation(name_stiffness, req.stiffness, 0.2)


        rospy.loginfo("MoveHri: Move Arm ASKED rad:" + str(req.turn_rad)+", r or l :"+str(req.arm_l_or_r)+ "stiffness: "+str(req.stiffness))
        return []

    def pointAt(self, req):
        if req.move_head:
            #Parameters lookAt:
            # lookAt(const std::vector<float>& Position, const float& FractionMaxSpeed, const bool UseWholeBody)
            #   - Position - position 3D [x, y, z] in FRAME_TORSO.
            #   - FractionMaxSpeed - a fraction.
            #   - UseWholeBody - if true, use whole body constraints.
            self._tracker_service.lookAt([req.x, req.y, req.z],0.7,False)
        if req.move_arm:
            self.release_fix_pose_thread()
            if req.y > 0:
                name_stiffness = ["LArm"]
                name = name_stiffness[0]
                isRightArm=False
            else:
                name_stiffgotoCarryPose2ness = ["RArm"]
                name = name_stiffness[0]
                isRightArm = True

            self._motion.stiffnessInterpolation(name_stiffness, 0.8, 1.0)


            #Parameters pointAt:
            # pointAt(const std::string& Effector, const std::vector<float>& Position, const int& Frame, const float& FractionMaxSpeed)
            #   - Effector - effector name. Could be "Arms", "LArm", "RArm".
            #   - Position - position 3D [x, y, z].
            #   - Frame - position frame {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2}.
            #   - FractionMaxSpeed - a fraction.
            self._tracker_service.pointAt(name, [req.x, req.y, req.z],0, 0.7)

            #Release if pose_duration is positive or equal to zero
            if req.pose_duration > 0.0:
                rospy.sleep(req.pose_duration)
                self.releaseArm(isRightArm,0.6)
            else:
                rospy.logwarn("Pepper arm still pointing: call release_arms service with 0.6 stiffness to release")
        return []

    def releaseArm(self,isRightArm,stiffness):
        self.release_fix_pose_thread()
        if isRightArm:
            name_angle = ["RShoulderPitch", "RShoulderRoll"]
            name_stiffness = ["RArm"]
        else:
            name_angle = ["LShoulderPitch", "LShoulderRoll"]
            name_stiffness = ["LArm"]
        # move arm at rest position
        angles = [1.6, 0]
        times = [1.0, 5.0]
        isAbsolute = True
        self._motion.angleInterpolation(name_angle, angles, times, isAbsolute)
        # set stiffness to 0.0
        self._motion.stiffnessInterpolation(name_stiffness, stiffness, 3.0)


    def releaseArmSrv(self,req):
        self.releaseArm(True,req.stiffness)
        self.releaseArm(False,req.stiffness)
        return []



    def gotoCarryPose(self, req):
        #FIXME provide a seconde carry pose close to the robot legs
        leftArmEnable = True
        rightArmEnable = True
        if req.arm_l_or_r == "r":
            name_stiffness = ["RArm"]
            self._motion.stiffnessInterpolation(name_stiffness, 1.0, 1.0)
            self._motion.angleInterpolation("RHand", 0.0, 1.0, True)
            self._motion.angleInterpolation("RWristYaw", 1.73, 1.0, True)
            self._motion.angleInterpolation("RElbowYaw",  2.08, 1.0, True)
            self._motion.angleInterpolation("RElbowRoll", 1.48, 1.0, True)
            self._motion.angleInterpolation("RShoulderPitch",-0.47, 2.0, True)
            self._motion.angleInterpolation("RShoulderRoll", -0.09, 2.0, True)

            self._motion.angleInterpolation("RShoulderPitch", -1.52, 2.0, True)
            self._motion.angleInterpolation("RShoulderRoll", 0, 2.0, True)
            self._motion.angleInterpolation("RElbowYaw",  0.96, 1.0, True)

            joint_names = ["RHand", "RWristYaw","RElbowYaw" "RElbowRoll", "RShoulderPitch", "RShoulderRoll"]
            joint_angles = [0.0, 1.73, 2.08, 1.48, -0.47, -0.09]
            rightArmEnable = False

        else:
            name_stiffness = ["LArm"]
            #FIXME to be tested and modify !!!!
            self._motion.stiffnessInterpolation(name_stiffness, 1.0, 1.0)
            self._motion.angleInterpolation("LHand", 0.0, 1.0, True)
            self._motion.angleInterpolation("LWristYaw", -1.73, 1.0, True)
            self._motion.angleInterpolation("LElbowYaw", -2.08, 1.0, True)
            self._motion.angleInterpolation("LElbowRoll", -1.48, 1.0, True)
            self._motion.angleInterpolation("LShoulderPitch", -0.47, 2.0, True)
            self._motion.angleInterpolation("LShoulderRoll", -0.09, 2.0, True)

            self._motion.angleInterpolation("LShoulderPitch", -1.52, 2.0, True)
            self._motion.angleInterpolation("LShoulderRoll", 0, 2.0, True)
            self._motion.angleInterpolation("LElbowYaw", - 0.96 , 1.0, True)

            joint_names=["LHand","LWristYaw","LElbowYaw","LElbowRoll","LShoulderPitch","LShoulderRoll"]
            joint_angles=[0.0,-1.73,-2.08,-1.48,-0.47,-0.09]
            leftArmEnable = False


        if req.keep_pose:
            self._motion.setMoveArmsEnabled(leftArmEnable, rightArmEnable)
            #self.release_fix_pose_thread()
            #NO MORE NEED TO KEEP THE POSE --> disable arm pose for nav. instead
            # self.isEnd = False
            # self.keep_arm_pose_thread = threading.Thread(target=self.keepArmPose, args=(joint_names,joint_angles,name_stiffness,5,))
            # self.keep_arm_pose_thread.start()

        self._motion.stiffnessInterpolation(name_stiffness, req.stiffness, 0.2)
        return []


    def keepArmPose(self,joint_names,joint_angles,arm_name,check_frequency):
        rate = rospy.Rate(check_frequency)  # 10hz
        while not self.isEnd and not rospy.is_shutdown():
            # reset the stiffness of the arm
            self._motion.stiffnessInterpolation(arm_name, 0.8, 0.5)
            #0.2=fraction of the max speed
            self._motion.setAngles(joint_names, joint_angles, 0.2)
            rate.sleep()

    def release_fix_pose_thread(self):
        self._motion.setMoveArmsEnabled(True, True)
        if hasattr(self, 'keep_arm_pose_thread'):
            self.isEnd = True
            self.keep_arm_pose_thread.join(100)





    def turnToInterestPoint(self, req ):
        """
            Tourne le robot vers un interest point

            param: label de l'interest point vers lequel le robot doit se tourner
        """    

        """ InterestPoint.msg

            string label
            geometry_msgs/Pose pose
            int32 arm_position
            float32 head_pitch
            float32 head_yaw
        """

        resp = self._getPoint_service( req.itP_label )

        

        itP_x = resp.itP.pose.position.x
        itP_y = resp.itP.pose.position.y

        rospy.loginfo("[MoveTurnRobot] ASK turn to an interest point: %s " % str(req.itP_label) )
        robot_pose = self._getRobotCurrentPose(0)
 
        
        dist, angle = self._computeDistAngle(itP_x, itP_y, robot_pose)
        
        self._motion.moveTo(0, 0, angle)

        return angle




    def _conf_TurnToInterestPoint(self):
        self._tflistener = TransformListener()
        self._getPoint_service = rospy.ServiceProxy('get_InterestPoint', getitP_service)




    def _computeDistAngle(self, itP_x, itP_y, robot_pose):
        """
            Compute the euclidian distance to the target and the angle between robot orientation and its target
            Return a tupple (distance, angle) with distance and angle to the target
        """        
        rob_x = robot_pose.position.x
        rob_y = robot_pose.position.y
        roll, pitch, rob_angle = tf.transformations.euler_from_quaternion( (    
                                                                robot_pose.orientation.x,
                                                                robot_pose.orientation.y,
                                                                robot_pose.orientation.z,
                                                                robot_pose.orientation.w,
                                                            ) )
        distCurTarget = sqrt(pow(itP_x - rob_x, 2) + pow(itP_y - rob_y, 2) )
        angleCurTarget = atan2( itP_y - rob_y , itP_x - rob_x )
        angle = self._shortestAngleDiff(angleCurTarget, rob_angle)    

        return (distCurTarget, angle)




    def _shortestAngleDiff(self, th1, th2):
        """
            Returns the shortest angle between 2 angles in the trigonometric circle
        """ 
        print('------------->-------------> th1: %s , th2: %s' %(th1 ,th2))
        anglediff = fmod( (th1 - th2), 2*pi)        

        if anglediff < 0.0:
            if fabs(anglediff) > (2*pi + anglediff) :
                anglediff = 2*pi + anglediff
                
        else:
            if anglediff > fabs(anglediff - 2*pi) :
                anglediff = anglediff - 2*pi

        
        return anglediff



    def _getRobotCurrentPose(self,data):
        rospy.loginfo("[MoveTurnRobot] ASK to get current robot pose")
        now = rospy.Time.now()
        self._tflistener.waitForTransform("/map", "base_footprint", now, rospy.Duration(2))
        (trans, rot) = self._tflistener.lookupTransform("/map", "base_footprint", now)
        robotPose = Pose()
        robotPose.position.x = trans[0]
        robotPose.position.y = trans[1]
        robotPose.position.z = trans[2]

        quaternion = (rot[0], rot[1], rot[2], rot[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2] + data
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        robotPose.orientation.x = q[0]
        robotPose.orientation.y = q[1]
        robotPose.orientation.z = q[2]
        robotPose.orientation.w = q[3]

        return robotPose





if __name__ == "__main__":
    rospy.init_node('pepper_move_hri')
    ip=rospy.get_param('~ip',"192.168.42.200")
    port=rospy.get_param('~port',9559)

    MoveTurnRobot(ip,port)
    rospy.spin()
