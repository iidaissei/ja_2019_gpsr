#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
# Title: GPSR競技設計ROSノード
# Author: Issei Iida
# Date: 2019/11/27
# Memo: 2019JapanOpen仕様
#---------------------------------------------------------------------

# Python
import sys
# ROS
import rospy
from std_msgs.msg import String
from mimi_common_pkg.srv import ManipulateSrv
import smach_ros
import smach

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_action_client import *
from common_function import *


class Admission(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_admissiion'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: ADMISSION')
            #enterTheRoomAC(0.8)
            rospy.loginfo('Admission completed!')
            return 'finish_admissiion'
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
            navigationAC(coordinate_list)
            speak('I arrived operator position')
            return 'arrived'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class ListenOrder(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['listen_success',
                            'listen_failure',
                            'next_order'],
                output_keys = ['order_out'])
        #Value
        self.listen_count = 1
        self.result = []
        
    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: LISTEN_ORDER')
            #self.result = ActionPlan.execute()
            speak('Please give me a order')
            rospy.sleep(2.0)
            self.result = [['grasp','cup'],['speak', 'See you rater']]
            if self.listen_count <= 3:
                if self.result == 'failure':
                    rospy.loginfo('Listening Failed')
                    self.listen_count += 1
                    return 'listen_failure'
                else:
                    rospy.loginfo('Listening Success')
                    userdata.order_out = self.result
                    self.result = []
                    self.listen_count = 1
                    return 'listen_success'
            else:
                rospy.loginfo('Move to next order')
                return 'next_order'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class ExecuteAction(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['go',
                            'mani',
                            'search',
                            'speak',
                            'action_complete'],
                input_keys = ['order_in',
                              'e_position_in'],
                output_keys = ['e_position_out',
                               'e_data_out'])
        self.action_list = []
        self.action_count = 0

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: EXECUTE_ACTION')
            self.action_list = userdata.order_in
            print self.action_list
            if self.action_count < len(self.action_list):
                rospy.loginfo('ActionCount: ' + str(self.action_count + 1))
                action = self.action_list[self.action_count][0]
                userdata.e_data_out = self.action_list[self.action_count][1]
                self.action_count += 1
                if action is 'go':
                    return 'go'
                elif action is 'search':
                    return 'search'
                elif action is 'speak':
                    return 'speak'
                elif action is 'grasp':
                    return 'mani'
                else:
                    speak('nani siten nen')
            else:
                rospy.loginfo('All action completed')
                self.action_count = 0
                self.action_list = []
                userdata.e_position_out = userdata.e_position_in
                return 'action_complete'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class CheckPosition(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['operator',
                            'not_operator'],
                input_keys = ['c_position_in'],
                output_keys = ['c_position_out'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: CHECK_POSITION')
            if userdata.c_position_in == 'operator':
                rospy.loginfo('OperatorPosition')
                return 'operator'
            else:
                rospy.loginfo('Not OperatorPosition')
                return 'not_operator'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class OrderCount(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['not_complete',
                            'all_order_complete'])
        self.order_count = 1

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: ORDER_COUNT')
            if self.order_count <= 3:
                rospy.loginfo('Order num: ' + str(self.order_count))
                self.order_count += 1
                return 'not_complete'
            else:
                rospy.loginfo('All order completed!')
                return 'all_order_complete'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class Move(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['navi_success',
                            'navi_failure'],
                input_keys = ['position_in'],
                output_keys = ['position_out'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: MOVE')
            coord_list = searchLocationName(userdata.position_in)
            result = navigationAC(coord_list)
            result = 'success'
            if result == 'success':
                rospy.loginfo('Navigation success')
                userdata.position_out = userdata.position_in
                return 'navi_success'
            else:
                rospy.loginfo('Navigation failed')
                return 'navi_failure'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class Manipulation(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['mani_success',
                            'mani_failure'],
                input_keys = ['mani_in'])
        #Service
        self.mani_srv = rospy.ServiceProxy('/manipulation', ManipulateSrv)
        self.obj = ManipulateSrv()
        #Publisher
        #self.pub_give_req = rospy.Publisher()
        #self.pub_place_req = rospy.Publisher()

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: MANIPULATION')
            self.obj.target = userdata.mani_in
            result = self.mani_srv(self.obj.target)
            if result.result == True:
                rospy.loginfo('Manipulation success')
                return 'mani_success'
            else:
                rospy.loginfo('Manipulation failed')
                return 'mani_failure'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class Search(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['search_success',
                            'search_failure'],
                input_keys = ['search_in'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: SEARCH')
            # 人発見アクションを改造してここで使えるようにする
            result = 'success'
            if result == 'success':
                rospy.loginfo('Search success')
                return 'search_success'
            else:
                rospy.loginfo('Search failed')
                return 'search_failure'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class Speak(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['speak_success'],
                input_keys = ['speak_in'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: SPEAK')
            speak(userdata.speak_in)
            rospy.sleep(2.0)
            result = 'success'
            rospy.loginfo('Speak success')
            return 'speak_success'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class Exit(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['exit'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: EXIT')
            coord_list = searchLocationName('entrance')
            result = navigationAC(coord_list)
            rospy.loginfo('Exit success')
            return 'exit'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


def main():
    sm_top = smach.StateMachine(
            outcomes = ['finish_gpsr'])
    sm_top.userdata.position = 'null'
    with sm_top:

        smach.StateMachine.add(
                'ADMISSION',
                Admission(),
                transitions = {'finish_admissiion':'CHECK'})

        smach.StateMachine.add(
                'MOVE_TO_OPERATOR',
                MoveToOperator(),
                transitions = {'arrived':'LISTEN_ORDER'})

        smach.StateMachine.add(
                'LISTEN_ORDER',
                ListenOrder(),
                transitions = {'listen_success':'EXECUTE_ACTION',
                               'listen_failure':'LISTEN_ORDER',
                               'next_order':'CHECK'},
                remapping = {'order_out':'order'})

        smach.StateMachine.add(
                'EXECUTE_ACTION',
                ExecuteAction(),
                transitions = {'go':'MOVE',
                               'mani':'MANIPULATION',
                               'search':'SEARCH',
                               'speak':'SPEAK',
                               'action_complete':'CHECK'},
                remapping = {'order_in':'order',
                             'e_position_in':'position',
                             'e_position_out':'position',
                             'e_data_out':'action_data'})

        # 現在位置確認とオーダーカウントを行うState
        sm_check = smach.Concurrence(
                outcomes = ['all_order_complete',
                            'stay',
                            'move'],
                default_outcome = 'all_order_complete',
                input_keys = ['position'],
                outcome_map = {'stay':
                              {'CHECK_POSITION':'operator',
                               'ORDER_COUNT':'not_complete'},
                               'move':
                              {'CHECK_POSITION':'not_operator',
                               'ORDER_COUNT':'not_complete'}})

        with sm_check:
            smach.Concurrence.add(
                    'CHECK_POSITION',
                    CheckPosition(),
                    remapping = {'c_position_in':'position',
                                 'c_position_out':'position'})
            smach.Concurrence.add(
                    'ORDER_COUNT',
                    OrderCount())

        smach.StateMachine.add(
                'CHECK',
                sm_check,
                transitions = {'all_order_complete':'EXIT',
                               'stay':'LISTEN_ORDER',
                               'move':'MOVE_TO_OPERATOR'})

        smach.StateMachine.add(
                'MOVE',
                Move(),
                transitions = {'navi_success':'EXECUTE_ACTION',
                               'navi_failure':'CHECK'},
                remapping = {'position_in':'action_data',
                             'position_out':'position'})

        smach.StateMachine.add(
                'MANIPULATION',
                Manipulation(),
                transitions = {'mani_success':'EXECUTE_ACTION',
                               'mani_failure':'CHECK'},
                remapping = {'mani_in':'action_data'})

        smach.StateMachine.add(
                'SPEAK',
                Speak(),
                transitions = {'speak_success':'EXECUTE_ACTION'},
                remapping = {'speak_in':'action_data'})

        smach.StateMachine.add(
                'SEARCH',
                Search(),
                transitions = {'search_success':'EXECUTE_ACTION',
                               'search_failure':'CHECK'},
                remapping = {'search_in':'action_data'})

        smach.StateMachine.add(
                'EXIT',
                Exit(),
                transitions = {'exit':'finish_gpsr'})


    outcome = sm_top.execute()


if __name__ == '__main__':
    rospy.init_node('n_sm_gpsr', anonymous = True)
    main()
