#!/usr/bin/env python

import rospy
from FetchController import *
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from std_msgs.msg import Header, String


def ik_solve(limb, point, quaternion):
    pass
    # ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    # iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    # ikreq = SolvePositionIKRequest()

    # hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    # poses = {
    #     str(limb): PoseStamped(header=hdr,
    #         pose=Pose(position=point, orientation=quaternion))}
    # ikreq.pose_stamp.append(poses[limb])
    # try:
    #     #rospy.wait_for_service(ns, 0.5)
    #     resp = iksvc(ikreq)
    # except (rospy.ServiceException, rospy.ROSException), e:
    #     rospy.logerr("Service call failed: %s" % (e,))
    #     return 0
    # if (resp.isValid[0]):
    #     # Format solution into Limb API-compatible dictionary
    #     limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    #     return limb_joints
    # else:
    #     return 0

def ik_handler(limb, msg):
    pass
    # remove the moveToEEPose at the end, and convert each string to float
    # msg_split = msg.strip().split(' ')
    # if len(msg_split) < 3:
    #     return
    # x, y, z, qx, qy, qz, qw = [float(elem) for elem in msg_split[0:7]]
    # limb_joints = ik_solve(limb, Point(x, y, z), Quaternion(qx, qy, qz, qw))
    # if limb_joints:
    #     if limb == 'right':
    #         right_limb.set_joint_positions(limb_joints)
    #     else:
    #         left_limb.set_joint_positions(limb_joints)
    # else:
    #     ik_fail_pub.publish('f')

def gripper_handler(msg):
    global gripper

    if 'openGripper' in msg:
        rospy.loginfo('Opening gripper.')
        gripper.open()
        gripper.move(10.1, 10.1, 0)
    if 'closeGripper' in msg:
        rospy.loginfo('Closing gripper.')
        gripper.close()

def pose_request_callback(data):
    msg = data.data

    # call handle ik
    # ik_handler(limb, msg)

    # call gripper handler
    gripper_handler(msg)
    

def main():
    global ik_fail_pub, gripper

    rospy.init_node('ros_reality_ik_interface')
    gripper = FetchController()

    ik_fail_pub = rospy.Publisher('ros_reality_ik_status', String, queue_size=0)

    rospy.Subscriber('/forth_commands', String, pose_request_callback)

    rospy.spin()


if __name__ == '__main__':
    main()
