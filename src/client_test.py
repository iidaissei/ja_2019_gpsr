#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Python関係ライブラリ
import sys
#ROS関係ライブラリ
import rospy
from gpsr.srv import ActionPlan

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_action_client import *
from common_function import *

bc = BaseCarrier()

class test():
    def __init__(self):
        self.ac = actionlib.SimpleActionClient('exe_action_plan', ExeActionPlanAction)
        self.ac.wait_for_server()
        self.lsten_srv = rospy.ServiceProxy('/gpsr/actionplan', ActionPlan)

    def execute(self):
        goal = ExeActionPlanGoal()
        goal = self.lsten_srv()
        ac.send_goal(goal)
        ac.wait_for_result()
        result = ac.get_result()
        if result.data == 'success':
            rospy.loginfo("Success EnterTheRoom")
            ac.cancel_goal()
            return True
        else:
            rospy.loginfo("Failed EnterTheRoom")
            ac.cancel_goal()
            return False


if __name__ == '__main__':
    rospy.init_node('action_test', anonymous = True)
    main()
