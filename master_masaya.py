#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
from enter_room.srv import EnterRoom
from happymimi_navigation.srv import NaviLocation
from happymimi_manipulation_msgs.srv import ArmControl
from happymimi_manipulation_msgs.srv import RecognitionToGrasp
from std_msgs.msg import String as StrInt
from std_msgs.msg import String as StrTrg  #�g�p����ROS����ъ֘A����p�b�P�[�W����у��b�Z�[�W�^���C���|�[�g����

class Enter1(smach.Enter):
	def __init__(self):     #�e�N���X�ɁA�N���X������������init
		smach.Enter._init_(self, outcomes = ['done',])�@�@�@#outcome�͏o��
		self.enter = rospy.ServiceProxy("enter_room_server", EnterRoom)
		self.counter = 0

	def excute(self, userdate):�@#��Ԃ̓�����`����execute 
		self.enter(0.3, 0.2)     #0.3m�̋�����0.2�̑��x�Ői��
		return 'done'

class Navigation1(smach.Navigation):
	def __init__(self, outcomes = ['done']):
		self.navi = rospy.ServicePoxy("/navi_location_server", NaviLocation)

	def excute(self, userdate):
		self.navi('bin')    #bin ���ĂƂ��܂Ői�ނ���
		return 'done'

class Navigation2(smach.Navigation):
	def __init__(self, outcomes = ['done']):
		self.navi = ros.ServiceProxy("/navi_location_server", NaviLocation)

	def excute(self, userdate):
		self.navi('Tall table')   #Tall table ���ĂƂ��܂Ői�ނ���
		return 'done'

class Navigation3(smach.Navigation):
	def __init__(self, outcomes = ['done']):
		self.grasp_cup = rospy.ServiceProxy('/recognition_to_grasping', RecognitionToGrasping)

	def excute(self, userdate):
		so = StrInt("center", 0)  
        self.grasp_cup(target_name='cup', sort_option=so)  #cup��͂�
		return 'done'

class Navigation4(smach.Navigation):
	def __init__(self, outcomes = ['done']):
		self.navi = ros.ServiceProxy("/navi_location_server", NaviLocation)

	def excute(self, userdate):
		self navi('oprator')   #opratpr ���ĂƂ��܂ňړ�
		return 'done'

class Navigation5(smach.Navigation):
	def __init__(self, outcomes = ['done']):
		self.give = rospy.ServiceProxy('/servo/arm', StrTrg)

	def excute(self, userdate):
		self.give('give')
		return 'done'

def main():
    rospy.init_node('smach_somple1')


    sm_top = smach.StateMachine(outcomes=['succeeded'])
    with sm_top:
        smach.StateMachine.add('ENTER1', Enter1(), transitions={'done':'NAVIGATION1'})
        smach.StateMachine.add('NAVIGATION1', Navigation(), transitions={'done':'NAVIGATION2'})
		smach.StateMachine.add('NAVIGATION2', Navigation2(), transitions={'done':'NAVIGATION3'})
		smach.StateMachine.add('NAVIGATION3', Navigation3(), transitions={'done':'NAVIGATION4'})
		smach.StateMachine.add('NAVIGATION4', Navigation4(), transitions={'done':'NAVIGATION5'})
		smach.StateMachine.add('NAVIGATION5', Navigation5(), transitions={'done':'succeeded'})

    outcome = sm_top.execute()

if __name__ == '__main__':
    main()