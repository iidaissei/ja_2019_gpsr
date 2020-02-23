#!/usr/bin/env python
# -*- coding: utf-8 -*-
#----------------------------------------------------------
# Title: GPSR競技設計用ROSノード
# Author: Issei Iida
# Date: 2020/02/13
# Memo: Version 2.2.0
#----------------------------------------------------------

# Python
import sys
# ROS
import rospy
import smach
import smach_ros
# Message/Service
from std_msgs.msg import String
from std_srvs.srv import Trigger
from gpsr.srv import ActionPlan
from mimi_common_pkg.srv import ManipulateSrv

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_action_client import *
from common_function import *


class Enter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['enter_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: ENTER')
        speak('Start gpsr')
        # enterTheRoomAC(0.8)
        return 'enter_finish'


class DecideMove(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['decide_finish', 'all_cmd_finish'],
                             input_keys = ['cmd_count_in'])
        # Subscriber
        self.posi_sub = rospy.Subscriber('/navigation/move_place', String, self.currentPosiCB)
        # Value
        self.coord_list = searchLocationName('operator')
        self.current_position = 'none'

    def currentPosiCB(self, data):
        self.current_position = data.data

    def execute(self, userdata):
        rospy.loginfo('Executing state: DECIDE_MOVE')
        print self.current_position
        if userdata.cmd_out_in == 3:
            speak('All command success')
            return 'all_cmd_finish'
        elif self.current_position != 'operator':
            navigationAC(self.coord_list)
            speak('I arrived operator position')
            return 'decide_finish'
        else:
            return 'decide_finish'


class ListenCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['listen_success',
                                         'listen_failure',
                                         'next_cmd'],
                             output_keys = ['cmd_out', 'cmd_count_out'])
        # ServiceProxy
        self.listen_srv = rospy.ServiceProxy('/gpsr/actionplan', ActionPlan)
        # Value
        self.listen_count = 1
        self.cmd_count = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN_COMMAND')
        if self.listen_count <= 3:
            speak('CommandNumber is ' + str(self.cmd_count))
            speak('ListenCount is ' + str(self.listen_count))
            speak('Please instruct me')
            result = self.listen_srv()
            if result.result:
                self.listen_count = 1
                self.cmd_count += 1
                userdata.cmd_out = result
                userdata.cmd_count_out = self.cmd_count
                return 'listen_success'
            else:
                self.listen_count += 1
                speak("Sorry, I could't listen")
                return 'listen_failure'
        else:
            speak("I couldn't understand the instruction")
            self.listen_count = 1
            self.cmd_count +=1
            userdata.cmd_count_out = self.cmd_count
            return 'next_cmd'


class ExeAction(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['action_success',
                                         'action_failure'],
                             input_keys = ['cmd_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: EXE_ACTION')
        action = userdata.cmd_in.action
        data = userdata.cmd_in.data
        print data
        print action
        result = exeActionPlanAC(action, data)
        if result:
            speak('Action success')
            return 'action_success'
        else:
            speak('Action failed')
            return 'action_failure'


class Exit(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['exit_finish'])
        # Value
        self.coord_list = searchLocationName('entrance')

    def execute(self, userdata):
        rospy.loginfo('Executing state: EXIT')
        result = navigationAC(self.coord_list)
        speak('Finish GPSR')
        return 'exit_finish'


def main():
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    with sm_top:
        smach.StateMachine.add(
                'ENTER',
                Enter(),
                transitions = {'enter_finish':'DECIDE_MOVE',
                               'all_cmd_finish':'EXIT'})

        smach.StateMachine.add(
                'DECIDE_MOVE',
                DecideMove(),
                transitions = {'decide_finish':'LISTEN_COMMAND'},
                remapping = {'cmd_count_in':'cmd_count'})

        smach.StateMachine.add(
                'LISTEN_COMMAND',
                ListenCommand(),
                transitions = {'listen_success':'EXE_ACTION',
                               'listen_failure':'LISTEN_COMMAND',
                               'next_cmd':'DECIDE_MOVE'},
                remapping = {'cmd_out':'cmd',
                             'cmd_count_out':'cmd_count'})

        smach.StateMachine.add(
                'EXE_ACTION',
                ExeAction(),
                transitions = {'action_success':'DECIDE_MOVE',
                               'action_failure':'DECIDE_MOVE'},
                remapping = {'cmd_in':'cmd'})

        smach.StateMachine.add(
                'EXIT',
                Exit(),
                transitions = {'exit_finish':'finish_sm_top'})

    outcome = sm_top.execute()


if __name__ == '__main__':
    rospy.init_node('sm_gpsr', anonymous = True)
    main()
