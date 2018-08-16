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

        request = split_msg[0]

        if request == "openGripper":
            self.fetch.open()

        elif request == "closeGripper":
            self.fetch.close()

        elif request == "move":
            coords = self.__get_coords(split_msg[1])
            self.fetch.move(coords[0], coords[1])

        elif request == "moveTo":
            coords = self.__get_coords(split_msg[1])
            self.fetch.move_to(coords[0], coords[1])

        elif request == "rotate":
            self.fetch.rotate(float(split_msg[1]))

        elif request == "rotateTo":
            self.fetch.rotate_to(float(split_msg[1]))

        elif request == "cancelMove":
            self.fetch.cancel_move()


if __name__ == '__main__':
    _ = IKInterface()