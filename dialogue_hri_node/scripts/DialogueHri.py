#!/usr/bin/env python  

import qi
import argparse
import sys
import time
import rospy
from collections import namedtuple
import json
import actionlib
from robocup_msgs.msg import gm_bus_msg
from dialogue_hri_actions.msg import DialogueSendSignalAction,DialogueStartScenarioAction

######### Command to Test
## 
## rostopic pub /gm_bus_command robocup_msgs/gm_b_msg "{'action': 'TTS', 'action_id': '1', 'payload': '{\"txt\":\"I am alive, test for pepper TTS of HRI module\",\"lang\":\"English\", \"mode\":\"NO_WAIT_END\"}' , 'result': 0}"
#########


class DialogueHri:
    TTS_ACTION="TTS"
    NO_WAIT_END_MODE="NO_WAIT_END"
    WAIT_END_MODE="WAIT_END"
    WAIT_END_STATUS="WAIT_END_STATUS"
    NONE_STATUS="NONE_STATUS"
    _status=NONE_STATUS
    _currentOrder=None
    _session=None
    _memory=None
    _tts=None
    _maxWaitTimePerCall=60
    _timeout_checker=False
    _t_timer=''
    def __init__(self,ip,port):
        self._ip=ip
        self._port=port
        while not self.configureNaoqi() and not not rospy.is_shutdown():
            rospy.sleep(0.5)
        self.configure()

    def configureNaoqi(self):
        self._session = qi.Session()
        try:
            self._session.connect("tcp://" + ip + ":" + str(port))
        except RuntimeError:
            rospy.logerr("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) +".\n"
                   "Please check your script arguments. Run with -h option for help.")
            return False


        self._memory = self._session.service("ALMemory")


        self.subscriber2 = self._memory.subscriber("ALTextToSpeech/Status")
        self.subscriber2.signal.connect(self.onTextStatus)
        return True

    def configure(self):
        

        # create action server and start it
        self._actionStartServer = actionlib.SimpleActionServer('dialogue_hri_start', DialogueStartScenarioAction, self.executeDialogueStartActionServer, False)
        self._actionStartServer.start()

        self._actionSignalServer = actionlib.SimpleActionServer('dialogue_hri_signal', DialogueSendSignalAction, self.executeDialogueSignalActionServer, False)
        self._actionSignalServer.start()

        self._status=self.NONE_STATUS

  
             
    def onTextStatus(self,status):
        print str(status)+'\n'
        if self._status==self.WAIT_END_STATUS:
             if status[1]=='done':
                self._currentOrder.result=3
                rospy.loginfo("TTS: text Done")
                self._gm_bus_pub.publish(self._currentOrder)
                self._status=self.NONE_STATUS
                self._currentOrder=None

    def processPayload(self,payload):
        try:
            #rospy.logwarn("payload: %s",str(payload))
            jsonObject = json.loads(payload, object_hook=lambda d: namedtuple('X', d.keys())(*d.values()))
            #rospy.logwarn("jsonObject: %s",str(jsonObject))
            jsonObject.txt
            jsonObject.mode
            jsonObject.lang
            #rospy.logwarn("Object ready txt:%s",str(jsonObject.txt))
            return jsonObject
        except Exception as e:
            rospy.logwarn("Unable to load TTS payload: %s" % e)
            return None
  



    def executeDialogueStartActionServer(self, goal):
        isActionSucceed=False
        try:
           
            try :
                #TODO SEND START SIGNAL
                
                isActionSucceed=True
            except RuntimeError as e:
                rospy.logwarn("Error occurs when sending signal:"+str(e))
        except Exception as e:
            rospy.logwarn("unable to find or launch function corresponding to the action %s:, error:[%s]",str(goal.action), str(e))
        if isActionSucceed:
            self._actionServer.set_succeeded()
        else:
            self._actionServer.set_aborted()


    def executeDialogueSignalActionServer(self, goal):
        isActionSucceed=True
        try:
            # call the process associating to the action
            # caution msg is publish to the gm_answer... needed ??
            

            isWaitForResult=False
            if self.WAIT_END_MODE == goal.mode:
                isWaitForResult=True

            #start text to speech
            try :
                rospy.loginfo("TTS: text to say:%s",str(goal.txt))
                if isWaitForResult:
                    self._status=self.WAIT_END_STATUS
                self._tts.say(goal.txt, goal.lang)
            except RuntimeError:
                rospy.logwarn(str(goal.lang)+" language is not installed, please install it to have a "+str(goal.lang)+" pronunciation.")
                self._tts.say(goal.txt, "English")
            if not isWaitForResult:
                isActionSucceed=True
            else:
                # set a timer
                # check if event trigged if yes return success
                self.timeout_checker=False
                self._t_timer = Timer(_maxWaitTimePerCall, self._timeout_checker)
                self._t_timer.start()
                while not self.timeout_checker:
                    rospy.sleep(0.1)
                isActionSucceed=True
        except Exception as e:
            rospy.logwarn("unable to find or launch function corresponding to the action %s:, error:[%s]",str(goal.action), str(e))
        if isActionSucceed:
            self._actionServer.set_succeeded()
        else:
            self._actionServer.set_aborted()



if __name__ == "__main__":
    rospy.init_node('pepper_dialogue_hri')
    ip=rospy.get_param('~ip',"192.168.0.147")
    port=rospy.get_param('~port',9559)
   
    
    DialogueHri(ip,port)
    rospy.spin()

