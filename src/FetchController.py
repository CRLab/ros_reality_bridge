#!/usr/bin/env python


# import select
# import sys
# import termios
from threading import Lock
# import tty

import math
import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import JointState


class FetchController(object):

    OPEN_POSITION = 0.1
    CLOSED_POSITION = 0

    def __init__(self):
        self._lock = Lock()

        self.max_effort = 20.0
        self.position = None
        self._sub_pos = rospy.Subscriber('joint_states', JointState, self.__set_state)

        # setup controller clients
        # self.client = actionlib.SimpleActionClient('gripper_controller/gripper_action', GripperCommandAction)
        self.client2 = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # wait for clients
        rospy.loginfo('Waiting for controllers...')
        # self.client.wait_for_server()
        self.client2.wait_for_server()
        rospy.loginfo('...connected to controllers.')

    def __set_state(self, joint_state):
        l_gripper_finger_pos = None
        r_gripper_finger_pos = None
        for joint, pos in zip(joint_state.name, joint_state.position):
            if joint == 'l_gripper_finger_joint':
                l_gripper_finger_pos = pos
            if joint == 'r_gripper_finger_joint':
                r_gripper_finger_pos = pos
        with self._lock:
            self.position = l_gripper_finger_pos + r_gripper_finger_pos

    def __set_position(self, position):
        goal = GripperCommandGoal()
        goal.command.max_effort = self.max_effort
        goal.command.position = position

        # Fill in the goal here
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))
        res = self.client.get_result()
        with self._lock:
            self.position = res.position

    def move(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = math.sin(theta / 2.0)
        move_goal.target_pose.pose.orientation.w = math.cos(theta / 2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        self.client2.send_goal(move_goal)
        self.client2.wait_for_result()


    def open(self):
        self.__set_position(self.OPEN_POSITION)

    def close(self):
        self.__set_position(self.CLOSED_POSITION)

