#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
#Title: 2019 RoboCupJapanOpen GPSRのマスター用ROSノード
#Author: Issei Iida 
#Date: 2019/10/13
#Memo:
#--------------------------------------------------------------------

#Python関連ライブラリ
import sys
#ROS関連ライブラリ
import rospy
from std_msgs.msg import String
import smach_ros
import smach

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_action_client import approachPersonAC, enterTheRoomAC
from common_function import speak, searchLocationName


class Admission(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['door_is_open'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: ADMISSION')
            enterTheRoomAC()
            rospy.loginfo('Admission completed!')
            return 'door_is_open'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class MoveToOperator(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['arrived'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: MOVE_TO_OPERATOR')
            coordinate_list = searchLocationName('operator')
            print coordinate_list
            navigationAC(coordinate_list)
            speak('I arrived operator position')
            rospy.sleep(2.0)
            return 'arrived'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass



class CheckCurrentPosition(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['operator',
                            'not_operator'],
                input_keys = ['c_current_posi_in'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: CHECK_CURRENT_POSITION')
            if userdata.c_current_posi_in == 'operator':
                rospy.loginfo('OperatorPosition')
                return 'operator'
            else:
                rospy.loginfo('Not OperatorPosition')
                return 'not_operator'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class CheckOrderCount(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['not_order_complete',
                            'all_order_complete'])
        self.order_count = 1

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: CHECK_ORDER_COUNT')
            if self.order_count <= 3:
                rospy.loginfo('Order Number: ' + str(self.order_count))
                self.order_count += 1
                return 'not_order_complete'
            else:
                rospy.loginfo('All Order Finished!')
                return 'all_order_complete'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class ListenOrder(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['listening_success',
                            'next_order'],
                output_keys = ['l_order_list_out'])
        
    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: LISTEN_ORDER')
            #デバッグ用
            result = ActionPlan.execute()
            if result == 'failure':
                rospy.loginfo('Listening Failed')
                return 'next_order'
            else:
                rospy.loginfo('Listening Success')
                userdata.l_order_list_out = result
                result = []
                return 'listening_success'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class ActionCount(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['exe_action',
                            'all_act_success'],
                input_keys = ['a_order_list_in'],
                output_keys = ['a_current_posi_out',
                               'exe_action_out'])
        self.action_list = []
        self.action_counter = 0

    def execute(self, userdata):
        try:
            self.action_list = userdata.a_order_list_in
            if self.action_counter <= len(self.action_list):
                rospy.loginfo('ActionCount: ' + self.action_counter)
                #カウントにしてその数をリストの要素数として利用したい
                userdata.exe_action_out = self.action_list[self.action_counter]
                self.action_counter += 1
                return 'start_action'
            else:
                rospy.loginfo('All action completed')
                self.action_counter = 0
                self.action_list = []
                #デバッグ用
                userdata.a_current_posi_out = 'operator'
                return 'all_act_success'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class ExeAction(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['act_finish'],
                input_keys = ['exe_action_in'])
        self.action_list = []

    def execute(self, userdata):
        try:
            self.action_list = userdata.exe_action_in
            rospy.loginfo('Action name: ' + self.action_list[0])
            if self.action_list[0] == 'go':
                result = navigationAC(self.action_list[1])
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')

def main():
    speak('start GPSR')
    sm = smach.StateMachine(
            outcomes = ['finish_gpsr'])
    sm.userdata.current_posi = 'Null'
    with sm:
        smach.StateMachine.add(
                'ADMISSION',
                DetectDoorOpen(),
                transitions = {'door_is_open':'CHECK'})

        smach.StateMachine.add(
                'MOVE_TO_OPERATOR',
                MoveToOperator(),
                transitions = {'arrived':'LISTEN_ORDER'})

        smach.StateMachine.add(
                'MOVE',
                Move(),
                transitions = {'arrived':'LISTEN_ORDER'})

        smach.StateMachine.add(
                'MANIPULATION',
                Manipulation(),
                transitions = {'mani_success':'EXE_ACTION'})

        #現在位置とオーダー数を確認するState
        sm_check = smach.Concurrence(
                outcomes = ['all_order_complete',
                            'stay',
                            'move'],
                default_outcome = 'all_order_complete',
                input_keys = ['current_posi'],
                outcome_map = {'stay':
                              {'CHECK_CURRENT_POSITION':'operator',
                               'CHECK_ORDER_COUNT':'not_order_complete'},
                               'move':
                              {'CHECK_CURRENT_POSITION':'not_operator',
                               'CHECK_ORDER_COUNT':'not_order_complete'}})
        with sm_check:
            smach.Concurrence.add(
                    'CHECK_CURRENT_POSITION',
                    CheckCurrentPosition(),
                    remapping={'c_current_posi_in':'current_posi'})
            smach.Concurrence.add(
                    'CHECK_ORDER_COUNT',
                    CheckOrderCount())

        smach.StateMachine.add(
                'CHECK',
                sm_check,
                transitions={'all_order_complete':'finish_gpsr',
                             'stay':'LISTEN_ORDER',
                             'move':'MOVE_TO_OPERATOR'})

        smach.StateMachine.add(
                'LISTEN_ORDER',
                ListenOrder(),
                transitions={'listening_success':'ACT',
                             'next_order':'LISTEN_ORDER'},
                remapping={'l_order_list_out':'order_list'})

        smach.StateMachine.add(
                'ACTION_COUNT',
                ActionCount(),
                transitions={'start_action':'EXE_ACTION',
                             'all_act_success':'CHECK'},
                remapping={'a_order_list_in':'order_list',
                           'a_current_posi_out':'current_posi',
                           'exe_action_out':'exe_action'})

    outcome = sm.execute()


if __name__ == '__main__':
    rospy.init_node('sm_gpsr', anonymous = True)
    main()
