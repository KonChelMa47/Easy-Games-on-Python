#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
from enter_room.srv import EnterRoom
from happymimi_navigation_srv import NaviLocation
from happymimi_manipulation_msgs.srv import RecognitionToGrasping
from happymimi_manipulation_msgs.srv import ArmControl
from happymimi_msgs.msg import StrInt
from happymimi_msgs.srv import StrTrg

self_voice = rospy.ServiceProxy("/tts", StrTrg)


class Enter_Room(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['enter_finish'])
        self.navi = rospy.ServiceProxy("/navi_location_server", NaviLocation)
        self.enter_srv = rospy.ServiceProxy("/enter_room_server", EnterRoom)

    def execute(self, userdata):
        self_voice("His welcome, Happy Mimi")
        self.enter_srv(1.0, 0.2)
        return 'enter_finish'
    
class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["navi_fin"])
        self.navi = rospy.ServiceProxy("/navi_location_server", NaviLocation)

    def execute(self, userdate):
        self_voice("Let's goooooo")
        rospy.sleep(1.0)
        self.navi('bin')
        return 'navi_fin'
    
class Manipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["mani_fin"])
        self.grasp_cup = rospy.ServiceProxy("/recognition_to_grasping", RecognitionToGrasping)
        self.navi = rospy.ServiceProxy("/navi_location_server", NaviLocation)
        self.give = rospy.ServiceProxy("/servo/arm", StrTrg)
        self.location = 'tall_table'
        self.count = 0

    def execute(self, userdata):
        if self.count == 1:
            self_voice("You need cup?, I'll bring you cup")
            self.navi(self.location)
            rospy.sleep(1.0)
            so = StrInt("center", 0)
            self.result = self.grasp_cup(targer_name='cup', sort_option=so).result
            self_voice("Finish")
        else:
            self_voice("One more time.You need cup, I'll bring you cup")
            self.navi(self.location)
            rospy.sleep(1.0)
            so = StrInt("center", 0)
            self.result = self.grasp_cup(targer_name='cup', sort_option=so).result
            self_voice("Finish")
        
        if self.result == True:
            self_voice("Well done")
            self.navi('operator')
            rospy.sleep(1.0)
            self_voice("Here you are")
            self.give('give')
            return "main_fin"
        elif self.result == False and self.count < 3:
            self.count += 1
            self_voice("Ah shit, here we go again")
            return "mani_not_finish"
        else:
            self_voice("I can't do that sorry")
            return "mani_finished"
        
def main():
    rospy.init_node("mimi_daniil")

    sm_top = smach.StateMachine(outcomes=["succeeded"])
    with sm_top:
        smach.StateMachine.add('ENTER',
                               Enter(),
                               transitions={'enter_finish':'NAVIGATION'})
        smach.StateMachine.add('NAVIGATION',
                               Navigation(),
                               transitions={'navi_fin':'MANIPULATION'})
        smach.StateMachine.add('MANIPULATION',
                               Manipulation(),
                               transitions={'mani_fin':'succeeded',"mani_not_finished":"MANIPULATION"})
    outcome = sm.top.execute()

if __name__ == '__main__':
    main()

