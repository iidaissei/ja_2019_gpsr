#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
# Title: GPSR競技設計ROSノード
# Author: Issei Iida
# Date: 2019/12/06
# Memo: 2019JapanOpen仕様
#---------------------------------------------------------------------

# Python
import sys
# ROS
import rospy
from std_msgs.msg import String, Bool
from mimi_common_pkg.srv import ManipulateSrv
from std_srvs.srv import Trigger
from gpsr.srv import ActionPlan
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
            # enterTheRoomAC(0.8)
            rospy.loginfo('Admission completed!')
            return 'finish_admissiion'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class MoveToOperator(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['arrived'],
                output_keys = ['m_position_out'])
        # Value
        # self.coordinate_list = searchLocationName('location_dict', 'operator')

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: MOVE_TO_OPERATOR')
            # self.coordinate_list = searchLocationName('operator')
            # navigationAC(self.coordinate_list)
            speak('I arrived operator position')
            userdata.m_position_out = 'operator'
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
        # ServiceProxy
        self.listen_srv = rospy.ServiceProxy('/gpsr/actionplan', ActionPlan)
        # Value
        self.listen_count = 1
        self.result = ActionPlan()

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: LISTEN_ORDER')
            speak('Please give me a order')
            self.result = self.listen_srv()
            if self.listen_count <= 3:
                if self.result.result == False:
                    rospy.loginfo('Listening Failed')
                    self.listen_count += 1
                    return 'listen_failure'
                elif self.result.result == True:
                    rospy.loginfo('Listening Success')
                    userdata.order_out = self.result
                    self.result = []
                    self.listen_count = 1
                    return 'listen_success'
            else:
                rospy.loginfo('Move to next order')
                self.listen_count = 1
                return 'next_order'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class CheckPosition(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['operator',
                            'not_operator'],
                input_keys = ['c_position_in'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: CHECK_POSITION')
            if userdata.c_position_in == 'operator':
                rospy.loginfo('Position OK!')
                return 'operator'
            else:
                rospy.loginfo('Not Operator')
                return 'not_operator'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class OrderCount(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['not_complete',
                                         'all_order_complete'])
        self.order_count = 0

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: ORDER_COUNT')
            if self.order_count < 3:
                self.order_count += 1
                rospy.loginfo('Order num: ' + str(self.order_count))
                return 'not_complete'
            else:
                rospy.loginfo('All order completed!')
                return 'all_order_complete'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class ExecuteAction(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['action_success',
                            'action_failure',
                            'action_complete'],
                input_keys = ['order_in'],
                output_keys = ['e_position_out'])
        # Service
        self.grasp_srv = rospy.ServiceProxy('/manipulation', ManipulateSrv)
        self.arm_srv = rospy.ServiceProxy('/change_arm_pose', ManipulateSrv)
        # Publisher
        self.pub_location = rospy.Publisher('/navigation/move_place', String, queue_size = 1)
        # Value
        self.obj = ManipulateSrv()
        self.order_data = []
        self.action_count = 0
        self.position = 'none'
        self.object_list = {'cup noodles':'cup',
                            'chip star':'bottle'}

    def selectAction(self, name, data):
        rospy.loginfo('Execute action: ' + name)
        if name == 'go':
            self.position = data
            # coord_list = searchLocationName('location_dict', data)
            # self.pub_location.publish(data)
            # result = navigationAC(coord_list)
            result = True
            pass
        elif name == 'grasp':
            if data in self.object_list:
                obj = self.object_list[data]
                print obj
                print 'good'
            else:
                print 'nonono'
            # result = self.grasp_srv(obj).result
            result = True
        elif name == 'place':
            result = self.arm_srv('place').result
        elif name == 'give':
            speak('Here you are')
            result = self.arm_srv('give').result
        elif name == 'search':
            result = localizeObjectAC(data)
        elif name == 'speak':
            speak(data)
            result = True
        rospy.loginfo('Action result: ' + str(result))
        return result

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: EXECUTE_ACTION')
            self.order_data = userdata.order_in
            if self.action_count < len(userdata.order_in.action):
                rospy.loginfo('ActionCount: ' + str(self.action_count + 1))
                result = self.selectAction(self.order_data.action[self.action_count],
                                           self.order_data.data[self.action_count])
                if result == True:
                    rospy.loginfo('Action Success')
                    userdata.e_position_out = self.position
                    self.action_count += 1
                    return 'action_success'
                elif result == False:
                    rospy.loginfo('Action Failed')
                    speak('Action failed')
                    speak('Sorry, I suspend this order')
                    self.action_count = 0
                    self.order_data = []
                    return 'action_failure'
            else:
                rospy.loginfo('All action completed')
                self.action_count = 0
                self.order_data = []
                return 'action_complete'
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
            # coord_list = searchLocationName('location_dict', 'entrance')
            # result = navigationAC(coord_list)
            speak('Finish gpsr')
            rospy.loginfo('Exit success')
            return 'exit'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


def main():
    sm_top = smach.StateMachine(
            outcomes = ['finish_gpsr'])
    sm_top.userdata.position = 'none'
    with sm_top:

        smach.StateMachine.add(
                'ADMISSION',
                Admission(),
                transitions = {'finish_admissiion':'CHECK'})

        smach.StateMachine.add(
                'MOVE_TO_OPERATOR',
                MoveToOperator(),
                transitions = {'arrived':'LISTEN_ORDER'},
                remapping = {'m_position_out':'position'})

        smach.StateMachine.add(
                'LISTEN_ORDER',
                ListenOrder(),
                transitions = {'listen_success':'EXECUTE_ACTION',
                               'listen_failure':'LISTEN_ORDER',
                               'next_order':'CHECK'},
                remapping = {'order_out':'order'})

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
                    remapping = {'c_position_in':'position'})
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
                'EXECUTE_ACTION',
                ExecuteAction(),
                transitions = {'action_success':'EXECUTE_ACTION',
                               'action_failure':'CHECK',
                               'action_complete':'CHECK'},
                remapping = {'order_in':'order',
                             'e_position_out':'position'})

        smach.StateMachine.add(
                'EXIT',
                Exit(),
                transitions = {'exit':'finish_gpsr'})

    outcome = sm_top.execute()


if __name__ == '__main__':
    rospy.init_node('sm_gpsr', anonymous = True)
    main()
