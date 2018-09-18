#!/usr/bin/env python

from threading import Lock
import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, PointHeadAction, PointHeadGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import JointState
from tf import TransformListener, transformations
import tf2_ros
import tf_conversions
import tf2_kdl
import geometry_msgs.msg
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import MoveItErrorCodes
import moveit_commander


class FetchController(object):

    OPEN_POSITION = 0.45
    CLOSED_POSITION = 0.005
    MAP = "/map"
    BASE = "/base_link"
    ARM = "arm_with_torso"

    def __init__(self):
        self._lock = Lock()

        self.tf = TransformListener()

        # setup gripper
        self.max_effort = 20.0
        self.position = None
        self._sub_pos = rospy.Subscriber('joint_states', JointState, self.__set_state)

        # moveit setup
        self.move_group = MoveGroupInterface(self.ARM, self.BASE)

        # setup controller clients
        self.gripper_client = actionlib.SimpleActionClient('gripper_controller/gripper_action', GripperCommandAction)
        self.move_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.mgc_gripper = moveit_commander.MoveGroupCommander('gripper')
        self.head_client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)  # For head tilt action

        # wait for clients
        rospy.loginfo('Waiting for controllers...')
        self.gripper_client.wait_for_server()
        self.move_client.wait_for_server()
        self.head_client.wait_for_server()
        rospy.loginfo('connected to controllers.')

    def __del__(self):
        self.move_group.get_move_action().cancel_all_goals()    # cancel moveit goals

    def __set_state(self, joint_state):
        l_gripper_finger_pos = None
        r_gripper_finger_pos = None
        for joint, pos in zip(joint_state.name, joint_state.position):
            if joint == 'l_gripper_finger_joint':
                l_gripper_finger_pos = pos
            if joint == 'r_gripper_finger_joint':
                r_gripper_finger_pos = pos
        with self._lock:
            if l_gripper_finger_pos is not None and r_gripper_finger_pos is not None:
                self.position = l_gripper_finger_pos + r_gripper_finger_pos

    # move the gripper to an end goal. pos = [x,y,z] rot = [x,y,z,w]
    def __move_gripper(self, pos, rot):
        # create pose
        pose = geometry_msgs.msg.Pose()
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]

        gripper_frame = 'wrist_roll_link'
        gripper_pose_stamped = PoseStamped()
        gripper_pose_stamped.header.frame_id = self.BASE
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = pose

        # Move gripper frame to the pose specified
        self.move_group.moveToPose(gripper_pose_stamped, gripper_frame)
        result = self.move_group.get_move_action().get_result()

        # error checking
        if result:
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                rospy.logerr("Arm goal in state: %s", self.move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")


    # set position of gripper
    def __set_position(self, position):
        # goal = GripperCommandGoal()
        # goal.command.max_effort = self.max_effort
        # goal.command.position = position

        # self.gripper_client.send_goal(goal)
        # self.gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))
        # res = self.gripper_client.get_result()
        # with self._lock:
        #     self.position = res.position

        self.mgc_gripper.set_joint_value_target([position]*2)
        p = self.mgc_gripper.plan()
        self.mgc_gripper.execute(p)

    # Point the head towards a point = [x,y,z]
    def __point_head(self, p):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = self.BASE
        
        # set pose
        goal.target.point.x = p[0]
        goal.target.point.y = p[1]
        goal.target.point.z = p[2]
        goal.min_duration = rospy.Duration(1.0)

        # execute and wait
        self.head_client.send_goal(goal)
        self.head_client.wait_for_result()
        
        
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
        # self.move_client.cancel_all_goals()

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
        # self.move_client.wait_for_result()

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

    # rotate the specified frame to to a specified angle
    def rotate_to(self, quat):
        rospy.loginfo('Rotating...')

        curr_trans, _ = self.__get_transform(self.MAP, self.BASE)

        # only rotate about z axis
        euler = transformations.euler_from_quaternion(quat)
        z_rot = transformations.quaternion_from_euler(0, 0, euler[2])

        self.__move_to(curr_trans, z_rot, self.MAP)

        rospy.loginfo('Done rotating.')

    def open(self):
        rospy.loginfo('Opening gripper...')
        self.__set_position(self.OPEN_POSITION)
        rospy.loginfo('Gripper open.')

    def close(self):
        rospy.loginfo('Closing gripper...')
        self.__set_position(self.CLOSED_POSITION)
        rospy.loginfo('Gripper closed.')

    # Stop all movement
    def cancel_move(self):
        self.move_client.cancel_all_goals()     # cancel base movement
        self.move_group.get_move_action().cancel_all_goals()    # cancel moveit goals

    # move the gripper to an end goal. pos = [x,y,z] rot = [x,y,z,w]
    def move_gripper(self, pos, rot):
        rospy.loginfo('Moving gripper...')
        self.__move_gripper(pos, rot)
        rospy.loginfo('Gripper moved.')

    def point_head(self, p):
        rospy.loginfo('Pointing head...')
        self.__point_head(p)
        rospy.loginfo('Head pointed.')