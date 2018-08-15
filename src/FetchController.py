#!/usr/bin/env python

from threading import Lock
import math
import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import JointState
from tf import TransformListener, transformations
import tf2_ros
import tf_conversions
import tf2_kdl


class FetchController(object):

    OPEN_POSITION = 0.1
    CLOSED_POSITION = 0
    MAP = "/map"
    BASE = "/base_link"

    def __init__(self):
        self._lock = Lock()

        self.tf = TransformListener()

        self.max_effort = 20.0
        self.position = None
        self._sub_pos = rospy.Subscriber('joint_states', JointState, self.__set_state)

        # setup controller clients
        self.gripper_client = actionlib.SimpleActionClient('gripper_controller/gripper_action', GripperCommandAction)
        self.move_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # wait for clients
        rospy.loginfo('Waiting for controllers...')
        self.gripper_client.wait_for_server()
        self.move_client.wait_for_server()
        rospy.loginfo('connected to controllers.')

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

        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))
        res = self.gripper_client.get_result()
        with self._lock:
            self.position = res.position

    # get transform of a frame relative to another
    def __get_transform(self, reference_frame, target_frame):
        translation_rotation = None
        try:
            self.tf.waitForTransform(reference_frame, target_frame,
                                      rospy.Time(0), timeout=rospy.Duration(1))
            translation_rotation = self.tf.lookupTransform(reference_frame, target_frame,
                                                            rospy.Time())
        except Exception as e1:
            try:
                tf_buffer = tf2_ros.Buffer()
                tf2_listener = tf2_ros.TransformListener(tf_buffer)
                transform_stamped = tf_buffer.lookup_transform(reference_frame, target_frame,
                                                               rospy.Time(0), timeout=rospy.Duration(1))
                translation_rotation = tf_conversions.toTf(tf2_kdl.transform_to_kdl(transform_stamped))
            except Exception as e2:
                rospy.logerr("get_transform::\n " +
                             "Failed to find transform from %s to %s" % (
                             reference_frame, target_frame,))

        return translation_rotation

    # move to a new position. trans = [x,y,z] and rot = [x,y,z,w]
    def __move_to(self, trans, rot, frame):

        # set a new position to move to
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = trans[0]
        move_goal.target_pose.pose.position.y = trans[1]
        move_goal.target_pose.pose.position.z = trans[2]

        move_goal.target_pose.pose.orientation.x = rot[0]
        move_goal.target_pose.pose.orientation.y = rot[1]
        move_goal.target_pose.pose.orientation.z = rot[2]
        move_goal.target_pose.pose.orientation.w = rot[3]
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # move
        self.move_client.send_goal(move_goal)
        self.move_client.wait_for_result()

    # Add the desired translation and rotation to the current pose. rotation in euler. Returns rotation in quaternion
    def __combine_pose(self, trans, rot, frame):

        # convert current rotation to euler
        curr_trans, curr_rot = self.__get_transform(frame, self.BASE)
        curr_euler = transformations.euler_from_quaternion(curr_rot)

        # combine with desired rotation and convert to quaternion
        new_trans = [trans[i] + curr_trans[i] for i in range(len(trans))]
        new_rot = [rot[i] + curr_euler[i] for i in range(len(rot))]
        quat_rot = transformations.quaternion_from_euler(new_rot[0], new_rot[1], new_rot[2])

        return new_trans, quat_rot

    ###################################################################################################################
    # PUBLIC
    ###################################################################################################################

    # move relative to current position
    def move(self, x, y):
        rospy.loginfo('Moving...')

        trans, rot = self.__combine_pose([x, y, 0], [0, 0, 0], self.BASE)
        self.__move_to(trans, rot, self.BASE)

        rospy.loginfo('Done moving.')

    # move to a given location on the map
    def move_to(self, x, y):
        rospy.loginfo('Moving...')

        rot = transformations.quaternion_from_euler(0, 0, 0)
        self.__move_to([x, y, 0], rot, self.MAP)

        rospy.loginfo('Done moving.')

    # rotate by theta
    def rotate(self, theta):
        rospy.loginfo('Rotating...')

        trans, rot = self.__combine_pose([0, 0, 0], [0, 0, theta], self.BASE)
        self.__move_to(trans, rot, self.BASE)

        rospy.loginfo('Done rotating.')

    # rotate to a specified angle
    def rotate_to(self, theta):
        rospy.loginfo('Rotating...')

        rot = transformations.quaternion_from_euler(0, 0, theta)
        self.__move_to([0, 0, 0], rot, self.MAP)

        rospy.loginfo('Done rotating.')

    def open(self):
        rospy.loginfo('Opening gripper...')
        self.__set_position(self.OPEN_POSITION)
        rospy.loginfo('Gripper open.')

    def close(self):
        rospy.loginfo('Closing gripper...')
        self.__set_position(self.CLOSED_POSITION)
        rospy.loginfo('Gripper closed.')
