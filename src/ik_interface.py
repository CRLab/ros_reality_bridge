#!/usr/bin/env python

import rospy
from FetchController import FetchController
from std_msgs.msg import Header, String


class IKInterface():

    def __init__(self):
        rospy.init_node('ros_reality_ik_interface')
        self.fetch = FetchController()

        rospy.Subscriber('/forth_commands', String, self.pose_request_callback)
        rospy.spin()

    def __get_coords(self, data):
        coords = data.split(",")
        return map(float, coords)

    def pose_request_callback(self, data):
        msg = data.data
        split_msg = msg.split("^")
        rospy.loginfo(split_msg)

        if split_msg is None or len(split_msg) == 0:
            rospy.logerr("Invalid message format")

        # Parse the message type and execute accordingly
        request = split_msg[0]

        if request == "openGripper":
            self.fetch.open()

        elif request == "closeGripper":
            self.fetch.close()

        elif request == "pointHead":
            self.fetch.point_head(self.__get_coords(split_msg[1]))

        elif request == "moveTo":
            coords = self.__get_coords(split_msg[1])
            self.fetch.move_to(coords[0], coords[1])

        elif request == "rotateTo":
            rot = self.__get_coords(split_msg[1]) 
            self.fetch.rotate_to(rot)

        elif request == "cancelMove":
            self.fetch.cancel_move()

        elif request == "moveGripper":
            pos = self.__get_coords(split_msg[1])
            rot = self.__get_coords(split_msg[2])
            self.fetch.move_gripper(pos, rot)

        elif request == "move":
            self.fetch.move(0.1,1)

        elif request == "rotate":
            self.fetch.rotate(1,1)
        elif request == "directMove":
            t = 0.05
            v,w = self.__get_coords(split_msg[1])
            v=v/4.0
            w=w/4.0
            self.fetch.move(t,v, -w)



if __name__ == '__main__':
    _ = IKInterface()