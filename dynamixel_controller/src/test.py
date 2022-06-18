#!/usr/bin/env python3

import rospy
from dynamixel_workbench_msgs.msg import DynamixelStateList


class MotorController():
    def __init__(self):
        rospy.Subscriber('/dynamixel_workbench/dynamixel_state', DynamixelStateList, self.getMotorStateCB)
        self.current_pose = [0]*6

    def getMotorStateCB(self, state):
        rospy.sleep(0.3)
        print(state.dynamixel_state)


if __name__ == '__main__':
    rospy.init_node('test_mc')
    mc = MotorController()
    rospy.spin()

