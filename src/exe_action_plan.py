#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: ActionPlanから行動を決定し実行するアクションサーバー
# Author: Issei Iida
# Date: 2020/02/07
# Memo:
#--------------------------------------------------------------------

# Python
import sys
# ROS
import rospy
from gpsr.srv import ActionPlan
from std_srvs.srv import Trigger
from  std_msgs.msg import String, Bool
from mimi_common_pkg.srv import ManipulateSrv
from smach_ros import ActionServerWrapper
from smach import StateMachine
import smach_ros
import smach

sys.path.insert(0, '/home/issei/catkin_ws/src/mimi_common_pkg/')
from common_action_client import *
from common_function import speak


class determineAction(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['move', 'mani', 'search', 'speak', 'all_finish'],
                             input_keys = ['goal_in', 'action_num'],
                             output_keys = ['action_output', 'data_output'])
        # Value
        self.action_count = 0
        self.action_name = 'none'
        self.action_data = 'none'
        # Param
        self.action_state = rosparam.get_param('/gpsr/action_state')

    def execute(self, userdata):
        rospy.loginfo('Executing state: DETERMINE_ACTION')
        self.action_count = userdata.action_num
        self.action_name = userdata.goal_in.action[self.action_count]
        self.action_data = userdata.goal_in.data[self.action_count]
        if self.action_count < len(userdata.order_in.action):
            userdata.action_output = self.action_name
            userdata.data_output = self.action_data
            self.action_count += 1
            speak('Action number ' + str(self.action_count))
            return self.action_state[self.action_name]
        else:
            speak('All action success')
            return 'all_finish'


class move(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['move_finish', 'move_failed'],
                             input_keys = ['action_in', 'data_in'])
        # Publisher
        self.pub_location = rospy.Publisher('/navigation/move_place', String, queue_size = 1)
 
    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVE')
        name = userdata.action_in
        data = userdata.data_in
        if name == 'go':
            coord_list = searchLocationName('location_dict', data)
            self.pub_location.publish(data)
            speak('I move to ' + data)
            result = navigationAC(coord_list)
            if result:
                return 'move_finish'
            else:
                return 'move_failed'
        else:
            return 'move_finish'


class mani(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['mani_finish', 'mani_failed'],
                             input_keys = ['action_in', 'data_in'])
        # Service
        self.grasp_srv = rospy.ServiceProxy('/manipulation', ManipulateSrv)
        self.arm_srv = rospy.ServiceProxy('/change_arm_pose', ManipulateSrv)
        # Param
        self.object_list = rosparam.get_param('/object_list')
 
    def execute(self, userdata):
        rospy.loginfo('Executing state: MANI')
        name = userdata.action_in
        data = userdata.data_in
        if name == 'grasp':
            speak('I grasp ' + data)
            result = self.grasp_srv(data).result
        elif name == 'place':
            speak('I put '+ data)
            result == self.arm_srv(name).result
        elif name == 'give':
            speak('Here you are')
            result = self.arm_srv(name).result
        rospy.loginfo('Result is ' + str(result))
        if result:
            return 'move_finish'
        else:
            return 'move_failed'


class search(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['search_finish', 'search_failed'],
                             input_keys = ['action_in', 'data_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: SEARCH')
        data = userdata.data_in
        result = localizeObjectAC(data)
        if result:
            return 'search_finish'
        else:
            return 'search_failed'


class speak(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['speak_finish', 'speak_failed'],
                             input_keys = ['action_in', 'data_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: SPEAK')
        data = userdata.data_in
        speak(data)
        return 'speak_finish'


def main():
    sm_top = StateMachine(
            outcomes = ['success',
                        'action_failed',
                        'preempted'],
            input_keys = ['goal_message'],
            output_keys = ['result_message'])
    userdata.action_num = 0
    with sm_top:
        StateMachine.add(
                'DETERMINE_ACTION',
                determineAction(),
                transitions = {'move':'MOVE',
                               'mani':'MANI',
                               'search':'SEARCH',
                               'speak':'SPEAK',
                               'all_finish':'success'},
                remapping = {'goal_in':'goal_message',
                             'action_output':'action_name'})

        StateMachine.add(
                'MOVE',
                move(),
                transitions = {'move_finish':'DETERMINE_ACTION',
                               'move_failed':'action_failed'},
                remapping = {'action_in':'action_name',
                             'data_in':'data_name'})

        StateMachine.add(
                'MANI',
                mani(),
                transitions = {'mani_finish':'DETERMINE_ACTION',
                               'mani_failed':'action_failed'},
                remapping = {'action_in':'action_name',
                             'data_in':'data_name'})

        StateMachine.add(
                'SEARCH',
                search(),
                transitions = {'search_finish':'DETERMINE_ACTION',
                               'search_failed':'action_failed'},
                remapping = {'action_in':'action_name',
                             'data_in':'data_name'})

        StateMachine.add(
                'SPEAK',
                speak(),
                transitions = {'speak_finish':'DETERMINE_ACTION',
                               'speak_failed':'action_failed'},
                remapping = {'action_in':'action_name',
                             'data_in':'data_name'})

    asw = ActionServerWrapper(
            'exe_action_plan',
            ExeActionPlanAction,
            wrapped_container = sm_top,
            succeeded_outcomes = ['success'],
            aborted_outcomes = ['action_failed'],
            preempted_outcomes = ['preempted'],
            goal_key = 'goal_message',
            result_key = 'result_message')

    asw.run_server()
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('exe_action_plan', anonymous = True)
    main()
