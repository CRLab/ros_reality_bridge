#!/usr/bin/env python


# import select
# import sys
# import termios
from threading import Lock
# import tty

import actionlib
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandGoal
import rospy
from sensor_msgs.msg import JointState


class GripperController(object):

    OPEN_POSITION = 0.1
    CLOSED_POSITION = 0

    def __init__(self):
        self._lock = Lock()

        self.max_effort = 20.0
        self.position = None
        self._sub_pos = rospy.Subscriber('joint_states', JointState, self._set_state)

        self.action_name = 'gripper_controller/gripper_action'
        self.client = actionlib.SimpleActionClient(self.action_name,
                                                   GripperCommandAction)
        rospy.loginfo('Waiting for gripper_controller...')
        self.client.wait_for_server()
        rospy.loginfo('...connected to gripper controller.')

    def _set_state(self, joint_state):
        l_gripper_finger_pos = None
        r_gripper_finger_pos = None
        for joint, pos in zip(joint_state.name, joint_state.position):
            if joint == 'l_gripper_finger_joint':
                l_gripper_finger_pos = pos
            if joint == 'r_gripper_finger_joint':
                r_gripper_finger_pos = pos
        with self._lock:
            self.position = l_gripper_finger_pos + r_gripper_finger_pos

    def set_position(self, position):
        goal = GripperCommandGoal()
        goal.command.max_effort = self.max_effort
        goal.command.position = position

        # Fill in the goal here
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))
        res = self.client.get_result()
        with self._lock:
            self.position = res.position

    def open(self):
        self.set_position(self.OPEN_POSITION)

    def close(self):
        self.set_position(self.CLOSED_POSITION)

