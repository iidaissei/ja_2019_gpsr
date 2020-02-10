#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Python関係ライブラリ
import sys
#ROS関係ライブラリ
import rospy
from mimi_common_pkg.msg import *
import actionlib

sys.path.insert(0, '/home/issei/catkin_ws/src/mimi_common_pkg/scripts/')
from common_action_client import *
from common_function import *


class ExeActionPlanAC():
    def __init__(self):
        self.ac = actionlib.SimpleActionClient('exe_action_plan', ExeActionPlanAction)
        self.goal = ExeActionPlanGoal()

    def execute(self, action_list, data_list):
        self.ac.wait_for_server()
        self.goal.action = action_list
        self.goal.data = data_list
        self.ac.send_goal(self.goal)
        self.ac.wait_for_result()
        result = self.ac.get_result()
        if result.data == 'success':
            rospy.loginfo("Success ExeActionPlan")
            rospy.sleep(0.3)
            self.ac.cancel_goal()
            return True
        else:
            rospy.loginfo("Failed ExeActionPlan")
            rospy.sleep(0.3)
            self.ac.cancel_goal()
            return False

   
if __name__ == '__main__':
    rospy.init_node('action_test', anonymous = True)
    t = test()
