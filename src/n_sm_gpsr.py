#!/usr/bin/env python
# -*- coding: utf-8 -*-
#----------------------------------------------------------
# Title: GPSR競技設計用ROSノード
# Author: Issei Iida
# Date: 2020/02/10
# Memo:
#----------------------------------------------------------

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
        smach.State.__init__(self, outcomes = ['finish_admissiion'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: ADMISSION')
        speak('Start gpsr')
        # enterTheRoomAC(0.8)
        rospy.loginfo('Admission completed!')
        return 'finish_admissiion'


class MoveToOperator(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['arrived'],
                             output_keys = ['m_position_out'])
        # Value
        # self.coordinate_list = searchLocationName('location_dict', 'operator')

    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVE_TO_OPERATOR')
        # self.coordinate_list = searchLocationName('operator')
        # navigationAC(self.coordinate_list)
        speak('I arrived operator position')
        userdata.m_position_out = 'operator'
        return 'arrived'


class ListenOrder(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['listen_success',
                                         'listen_failure',
                                         'next_order'],
                             input_keys = ['order_c_in'],
                             output_keys = ['order_out'])
        # ServiceProxy
        self.listen_srv = rospy.ServiceProxy('/gpsr/actionplan', ActionPlan)
        # Value
        self.listen_count = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN_ORDER')
        # speak('Please give me a order')
        # result = self.listen_srv()
        order_count = userdata.order_c_in
        if order_count == 4:
            speak('Finish all order')
            return 'next_order'
        elif self.listen_count <= 3:
            speak('OrderNumber is ' + str(order_count))
            speak('ListenNumber is ' + str(self.listen_count))
            speak('Please give me a order')
            result = self.listen_srv()
            if result.result:
                rospy.loginfo('Listening Success')
                userdata.order_out = result
                self.listen_count = 1
                return 'listen_success'
            else:
                rospy.loginfo('Listening Failed')
                speak("Sorry, I could't listen")
                self.listen_count += 1
                return 'listen_failure'
        else:
            rospy.loginfo('Move to next order')
            speak('Move to next order')
            self.listen_count = 1
            return 'next_order'


class CheckPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['operator',
                                         'not_operator'],
                             input_keys = ['c_position_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: CHECK_POSITION')
        if userdata.c_position_in == 'operator':
            rospy.loginfo('Position OK!')
            return 'operator'
        else:
            rospy.loginfo('Not Operator')
            return 'not_operator'


class OrderCount(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['not_complete',
                                         'all_order_complete'],
                             output_keys = ['order_c_out'])
        self.order_count = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state: ORDER_COUNT')
        if self.order_count < 3:
            self.order_count += 1
            rospy.loginfo('Order num: ' + str(self.order_count))
            userdata.order_c_out = self.order_count
            return 'not_complete'
        else:
            rospy.loginfo('All order completed!')
            return 'all_order_complete'


class ExecuteAction(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['action_success',
                            'action_failure'],
                input_keys = ['order_in'],
                output_keys = ['e_position_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: EXECUTE_ACTION')
        action = userdata.order_in.action
        data = userdata.order_in.data
        print data
        print action
        # result = exeActionPlan(action, data)
        result = False
        # if result.result:
        if result:
            rospy.loginfo('Action Success')
            return 'action_success'
        else:
            rospy.loginfo('Action Failed')
            return 'action_failure'


class Exit(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['exit'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: EXIT')
        # coord_list = searchLocationName('location_dict', 'entrance')
        # result = navigationAC(coord_list)
        speak('Finish gpsr')
        return 'exit'


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
                remapping = {'order_out':'order',
                             'order_c_in':'o_count'})

        # 現在位置確認とオーダーカウントを行うState
        sm_check = smach.Concurrence(
                outcomes = ['all_order_complete',
                            'stay',
                            'move'],
                default_outcome = 'all_order_complete',
                input_keys = ['position'],
                output_keys = ['o_count'],
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
                    OrderCount(),
                    remapping = {'order_c_out':'o_count'})

        smach.StateMachine.add(
                'CHECK',
                sm_check,
                transitions = {'all_order_complete':'EXIT',
                               'stay':'LISTEN_ORDER',
                               'move':'MOVE_TO_OPERATOR'})

        smach.StateMachine.add(
                'EXECUTE_ACTION',
                ExecuteAction(),
                transitions = {'action_success':'CHECK',
                               'action_failure':'CHECK'},
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
