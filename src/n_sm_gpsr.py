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
from std_msgs.msg import String, Bool
from mimi_common_pkg.srv import ManipulateSrv
from std_srvs.srv import Trigger
from ti_gpsr.msg import array
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
            speak('Start gpsr')
            enterTheRoomAC(0.8)
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
                outcomes = ['listen_success'
                            #'listen_failure',
                            #'next_order'
                            ],
                output_keys = ['order_out'])
        # Publisher
        self.pub_API = rospy.Publisher('/gpsrface', Bool, queue_size = 1)
        # ServiceProxy
        self.deki_srs = rospy.ServiceProxy('/service_call', Trigger)
        # Value
        self.listen_count = 1
        self.plan_1 = [['go','desk'],['grasp','cup'],['go','operator'],['give']]
        self.plan_2 = [['go','cupboard'],['grasp','cup'],['go','desk'],['place']]
        self.plan_3 = [['go','shelf'],['grasp','cup'],['go','operator'],['give']]

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: LISTEN_ORDER')
            speak('Please give me a order')
            result = self.deki_srs()
            print result
            while not rospy.is_shutdown():
                if result.message == '1':
                    userdata.order_out = self.plan_1
                    break
                elif result.message == '2':
                    userdata.order_out = self.plan_2
                    break
                elif result.message == '3':
                    userdata.order_out = self.plan_3
                    break
                else:
                    result = self.deki_srs()
            return 'listen_success'
            #if self.listen_count <= 3:
            #    if self.result == 'failure':
            #        rospy.loginfo('Listening Failed')
            #        speak('One more time Please')
            #        rospy.sleep(0.1)
            #        self.listen_count += 1
            #        return 'listen_failure'
            #    else:
            #        rospy.loginfo('Listening Success')
            #        userdata.order_out = self.result
            #        self.result = []
            #        self.listen_count = 1
            #        return 'listen_success'
            #else:
            #    rospy.loginfo('Move to next order')
            #    self.result = []
            #    return 'next_order'
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
                               'e_action_out',
                               'e_data_out'])
        self.order_data = []
        self.action_count = -1
        self.plan = []

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: EXECUTE_ACTION')
            self.plan = userdata.order_in
            print userdata.order_in
            # 失敗した時のアクションカウントの初期化処理が不十分。とりま以下の処理で解決してない。
            if self.plan != userdata.order_in:
                print 'New order start'
                self.action_count = -1
            elif self.action_count < len(userdata.order_in)-1:
                self.action_count += 1
                rospy.loginfo('ActionCount: ' + str(self.action_count + 1))
                userdata.e_action_out = userdata.order_in[self.action_count][0]
                action = userdata.order_in[self.action_count][0]
                print action
                if action is 'go':
                    userdata.e_data_out = userdata.order_in[self.action_count][1]
                    data = userdata.order_in[self.action_count][1]
                    print data
                    return 'go'
                elif action is 'search':
                    userdata.e_data_out = userdata.order_in[self.action_count][1]
                    return 'search'
                elif action is 'speak':
                    userdata.e_data_out = userdata.order_in[self.action_count][1]
                    return 'speak'
                elif action is 'grasp':
                    userdata.e_data_out = userdata.order_in[self.action_count][1]
                    return 'mani'
                elif action is 'place':
                    return 'mani'
                elif action is 'give':
                    return 'mani'
                else:
                    speak('nani siten nen')
            else:
                rospy.loginfo('All action completed')
                self.action_count = -1
                self.order_data = []
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
            if self.order_count <= 1:
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
        #Publisher
        self.pub_location = rospy.Publisher('/navigation/move_place', String, queue_size = 1)

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: MOVE')
            coord_list = searchLocationName(userdata.position_in)
            self.pub_location.publish(userdata.position_in)
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
                input_keys = ['action_in',
                              'data_in'])
        # Service
        self.mani_srv = rospy.ServiceProxy('/manipulation', ManipulateSrv)
        self.obj = ManipulateSrv()
        # Publisher
        self.pub_arm_req = rospy.Publisher('/arm/changing_pose_req', String, queue_size = 1)
        # Subscriber
        self.sub_arm_res = rospy.Subscriber('/arm/changing_pose_res', Bool, self.armChangeCB)
        # Value
        self.arm_result = False

    def armChangeCB(self, receive_msg):
        self.arm_result = receive_msg.data

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: MANIPULATION')
            rospy.loginfo('Start action: ' + str(userdata.action_in))
            self.obj.target= userdata.data_in
            if userdata.action_in == 'grasp':
                result = self.mani_srv(self.obj.target)
                if result.result == True:
                    rospy.loginfo('Manipulation success')
                    return 'mani_success'
                else:
                    rospy.loginfo('Manipulation failed')
                    return 'mani_failure'
            else:
                self.pub_arm_req.publish(userdata.action_in)
                self.arm_result = False
                while not rospy.is_shutdown() and self.arm_result == False:
                    rospy.loginfo('Waiting for arm_result')
                    rospy.sleep(1.0)
                return 'mani_success'
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
            result = localizeObjectAC(userdata.search_in[2])
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
            speak('Finsh gpsr')
            rospy.loginfo('Exit success')
            return 'exit'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


def main():
    sm_top = smach.StateMachine(
            outcomes = ['finish_gpsr'])
    sm_top.userdata.position = 'none'
    sm_top.userdata.action_name = 'none'
    sm_top.userdata.action_data = 'none'
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
                               #'listen_failure':'LISTEN_ORDER',
                               #'next_order':'CHECK'
                               },
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
                             'e_action_out':'action_name',
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
                remapping = {'data_in':'action_data',
                             'action_in':'action_name'})

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
