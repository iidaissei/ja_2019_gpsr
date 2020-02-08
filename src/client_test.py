#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Python関係ライブラリ
import sys
#ROS関係ライブラリ
import rospy
from gpsr.srv import ActionPlan
from ja_2019_gpsr.msg import *
import actionlib

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_action_client import *
from common_function import *

bc = BaseCarrier()

class test():
    def __init__(self):
        self.ac = actionlib.SimpleActionClient('exe_action_plan', ExeActionPlanAction)
        speak('start action')
        self.lsten_srv = rospy.ServiceProxy('/gpsr/actionplan', ActionPlan)
        self.ac.wait_for_server()
        goal = ExeActionPlanGoal()
        speak('Please give me a order')
        action_plan = self.lsten_srv()
        goal.action = action_plan.action
        goal.data = action_plan.data
        self.ac.send_goal(goal)
        self.ac.wait_for_result()
        result = self.ac.get_result()
        if result.data == 'success':
            rospy.loginfo("Success EnterTheRoom")
            self.ac.cancel_goal()
            return True
        else:
            rospy.loginfo("Failed EnterTheRoom")
            self.ac.cancel_goal()
            return False

   
if __name__ == '__main__':
    rospy.init_node('action_test', anonymous = True)
    t = test()
