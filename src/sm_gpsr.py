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
from ggi.srv import YesNo

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_action_client import *
from common_function import *


class Enter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['enter_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: ENTER')
        speak('Start GPSR')
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
        self.operator_coord = searchLocationName('operator')
        self.exit_coord = searchLocationName('entrance')
        self.current_position = 'none'

    def currentPosiCB(self, data):
        self.current_position = data.data

    def execute(self, userdata):
        rospy.loginfo('Executing state: DECIDE_MOVE')
        if userdata.cmd_count_in == 4:
            speak('Finish all command')
            speak('Move to exit')
            navigationAC(self.exit_coord)
            speak('Finish GPSR')
            return 'all_cmd_finish'
        elif self.current_position != 'operator':
            navigationAC(self.operator_coord)
            # speak('I arrived operator')
            return 'decide_finish'
        else:
            return 'decide_finish'


class ListenCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['listen_success',
                                         'listen_failure',
                                         'next_cmd'],
                             input_keys = ['cmd_count_in'],
                             output_keys = ['cmd_out',
                                            'cmd_count_out'])
        # ServiceProxy
        self.listen_srv = rospy.ServiceProxy('/gpsr/actionplan', ActionPlan)
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)
        # Value
        self.listen_count = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN_COMMAND')
        cmd_count = userdata.cmd_count_in
        if self.listen_count <= 3:
            speak('CommandNumber is ' + str(cmd_count))
            speak('ListenCount is ' + str(self.listen_count))
            speak('Please instruct me')
            result = self.listen_srv()
            if result.result:
                speak('Is this correct?')
                answer = self.yesno_srv().result
                if answer:
                    self.listen_count = 1
                    cmd_count += 1
                    userdata.cmd_out = result
                    userdata.cmd_count_out = cmd_count
                    return 'listen_success'
                else:
                    speak('Sorry')
                    self.listen_count += 1
                    return 'listen_failure'
            else:
                self.listen_count += 1
                speak("I could't listen")
                return 'listen_failure'
        else:
            speak("I couldn't understand the instruction")
            self.listen_count = 1
            cmd_count +=1
            userdata.cmd_count_out = cmd_count
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
            return 'action_success'
        else:
            return 'action_failure'


def main():
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    sm_top.userdata.cmd_count = 1
    with sm_top:
        smach.StateMachine.add(
                'ENTER',
                Enter(),
                transitions = {'enter_finish':'DECIDE_MOVE'})

        smach.StateMachine.add(
                'DECIDE_MOVE',
                DecideMove(),
                transitions = {'decide_finish':'LISTEN_COMMAND',
                               'all_cmd_finish':'finish_sm_top'},
                remapping = {'cmd_count_in':'cmd_count'})

        smach.StateMachine.add(
                'LISTEN_COMMAND',
                ListenCommand(),
                transitions = {'listen_success':'EXE_ACTION',
                               'listen_failure':'LISTEN_COMMAND',
                               'next_cmd':'DECIDE_MOVE'},
                remapping = {'cmd_out':'cmd',
                             'cmd_count_in':'cmd_count',
                             'cmd_count_out':'cmd_count'})

        smach.StateMachine.add(
                'EXE_ACTION',
                ExeAction(),
                transitions = {'action_success':'DECIDE_MOVE',
                               'action_failure':'DECIDE_MOVE'},
                remapping = {'cmd_in':'cmd'})

    outcome = sm_top.execute()


if __name__ == '__main__':
    rospy.init_node('sm_gpsr', anonymous = True)
    main()
