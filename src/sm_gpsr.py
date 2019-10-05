#!/usr/bin/env python
# -*- coding: utf-8 -*-

#--------------------------------------------------------------------
#Title: 2019 RoboCupJapanOpen GPSRのマスター用ROSノード
#Author: Issei Iida 
#Date: 2019/09/07
#Memo:
#--------------------------------------------------------------------

#Python関連ライブラリ
import sys
#ROS関連ライブラリ
import rospy
import roslib
import smach
import smach_ros
from std_msgs.msg import String

sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/scripts')
from common_function import *
from common_action_client import detectDoorOpenAC


class DetectDoorOpen(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['door_is_open'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: DETECT_DOOR_OPEN')
        result = detectDoorOpenAC()
        if result == 'success':
            #linear0.3を15回で80㌢前進
            for i in range(20):
                linearControl(0.3)
                rospy.sleep(0.2)
        else:
            pass
        return 'door_is_open'


class MoveToOperator(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['arrived'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVE_TO_OPERATOR')
        coordinate_list = searchLocationName('operator')
        print coordinate_list
        #startNavigation(coordinate_list)
        return 'arrived'


class CheckCurrentPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['operator','not_operator'],
                             input_keys=['c_current_posi_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: CHECK_CURRENT_POSITION')
        if userdata.c_current_posi_in == 'operator':
            rospy.loginfo('OperatorPosition')
            return 'operator'
        else:
            rospy.loginfo('Not OperatorPosition')
            return 'not_operator'


class CheckOrderCount(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['not_order_complete','all_order_complete'])
        self.order_count = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state: CHECK_ORDER_COUNT')
        if self.order_count <= 3:
            rospy.loginfo('Order Number: ' + str(self.order_count))
            self.order_count += 1
            return 'not_order_complete'
        else:
            rospy.loginfo('All Order Finished!')
            return 'all_order_complete'


class ListenOrder(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['listening_success', 'next_order'],
                             output_keys=['l_order_list_out'])
        self.order_list = []
        self.listen_count = 0
        
    #認識できたか否かのみ判断、聞き取りカウントは大西のほうで行う
    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN_ORDER')
        #デバッグ用
        listening_order = 'success'
        if listening_order == 'success':
            rospy.loginfo('Listening Success')
            #デバッグ用
            self.order_list = ['go','speak','place','pick']
            userdata.l_order_list_out = self.order_list
            self.order_list = []
            return 'listening_success'
        else:
            roslib.loginfo('Listening Failed')
            return 'next_order'

class Act(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['act_success','all_act_success'],
                             input_keys=['a_order_list_in'],
                             output_keys=['a_current_posi_out'])
        self.action_list = []
        self.action_counter = 0

    def execute(self, userdata):
        self.action_list = userdata.a_order_list_in
        if self.action_counter < len(self.action_list):
            rospy.loginfo('Action start')
            rospy.loginfo('Action is: <' + self.action_list[self.action_counter] + '>')
            self.action_counter += 1
            #カウントにしてその数をリストの要素数として利用したい
            return 'act_success'
        else:
            rospy.loginfo('All action completed')
            self.action_counter = 0
            self.action_list = []
            #デバッグ用
            userdata.a_current_posi_out = 'operator'
            return 'all_act_success'

def main():
    sm = smach.StateMachine(outcomes=['finish_gpsr'])
    sm.userdata.current_posi = 'Null'
    with sm:
        smach.StateMachine.add('DETECT_DOOR_OPEN',
                                DetectDoorOpen(),
                                transitions={'door_is_open':'CHECK'})

        smach.StateMachine.add('MOVE_TO_OPERATOR',
                                MoveToOperator(),
                                transitions={'arrived':'LISTEN_ORDER'})


        #現在位置とオーダー数を確認するState
        sm_check = smach.Concurrence(outcomes=['all_order_complete','stay','move'],
                                     default_outcome='all_order_complete',
                                     input_keys=['current_posi'],
                                     outcome_map={'stay':
                                                 {'CHECK_CURRENT_POSITION':'operator',
                                                  'CHECK_ORDER_COUNT':'not_order_complete'},
                                                 'move':
                                                 {'CHECK_CURRENT_POSITION':'not_operator',
                                                  'CHECK_ORDER_COUNT':'not_order_complete'}})
        with sm_check:
            smach.Concurrence.add('CHECK_CURRENT_POSITION',CheckCurrentPosition(),
                                  remapping={'c_current_posi_in':'current_posi'})
            smach.Concurrence.add('CHECK_ORDER_COUNT',CheckOrderCount())


        smach.StateMachine.add('CHECK',
                               sm_check,
                               transitions={'all_order_complete':'finish_gpsr',
                                            'stay':'LISTEN_ORDER',
                                            'move':'MOVE_TO_OPERATOR'})

        smach.StateMachine.add('LISTEN_ORDER',
                                ListenOrder(),
                                transitions={'listening_success':'ACT',
                                             'next_order':'LISTEN_ORDER'},
                                remapping={'l_order_list_out':'order_list'})

        smach.StateMachine.add('ACT',
                                Act(),
                                transitions={'act_success':'ACT',
                                             'all_act_success':'CHECK'},
                                remapping={'a_order_list_in':'order_list',
                                           'a_current_posi_out':'current_posi'})

    outcome = sm.execute()


if __name__ == '__main__':
    rospy.init_node('sm_gpsr', anonymous = True)
    main()
