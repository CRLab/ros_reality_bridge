#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import tf
# from tf import TransformListener, transformations
import tf2_ros
import tf_conversions
import tf2_kdl

# BASE = "/map"
BASE = "map"


def message_builder(link_dict):
    """
    Builds the message to send on the ROS network to the VR computer

    Returns:
        msg (string): the message
    """

    msg = ""
    for k, v in link_dict.iteritems():
        if v:
            trans, rot = v
            trans = [float("{0:.3f}".format(n)) for n in trans]
            rot = [float("{0:.3f}".format(n)) for n in rot]
            msg += '{}:{}^{};'.format(k, trans, rot)
    # remove spaces to save on space
    return msg.replace(' ', '')


# def get_transform(link, tf_listener):
    """
    update the link_dict with the position and rotation (transform) of the link relative to the base of the robot

    Params:
        link (string): name of the link to get transform of
    """
    # try:
    #     t = tf_listener.getLatestCommonTime(BASE, link)
    #     (trans, rot) = tf_listener.lookupTransform(BASE, link, t)
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
    #     rospy.logerr(e)
    #     return
    # return trans, rot

def get_transform(link, tf_buffer):
    """
    update the link_dict with the position and rotation (transform) of the link relative to the base of the robot

    Params:
        link (string): name of the link to get transform of
    """
    try:
        transform_stamped = tf_buffer.lookup_transform(BASE, link, rospy.Time(0))
        translation_rotation = tf_conversions.toTf(tf2_kdl.transform_to_kdl(transform_stamped))
    except Exception as e2:
        rospy.logerr("get_transform::\n " +
                     "Failed to find transform from %s to %s" % (
                     BASE, link,))

    return translation_rotation


def main():
    # initialize the ROS node
    rospy.init_node("unityNode", anonymous=False)

    # set up the publisher
    pub = rospy.Publisher("ros_unity", String, queue_size=0)
    # set up the tf listener
    # tf_listener = tf.TransformListener()

    tf_buffer = tf2_ros.Buffer()
    tf2_listener = tf2_ros.TransformListener(tf_buffer)

    # create a rate to sleep after every loop
    rate = rospy.Rate(60)

    trans = (0, 0, 0)
    rot = (0, 0, 0, 0)

    # sleep to give time for publishers and listener to initilize
    rospy.sleep(3)

    # dictionary to store the position and rotation of each link in the robot
    link_dict = dict()
    # import ipdb
    # ipdb.set_trace()
    
    # initialize keys in linkDict
    # for link in tf_listener.getFrameStrings():
    #     if 'reference' not in link:
    #         link_dict[link] = (trans, rot)
    print(tf_buffer.all_frames_as_string())
    for line in tf_buffer.all_frames_as_string().strip().split('\n'):
        print(line)
        link = line.split(' ')[1]
        link_dict[link] = (trans, rot)

    # import ipdb
    # ipdb.set_trace()

    # main loop. Updates the values for each link in link_dict and publish the values
    while not rospy.is_shutdown():
        for link in link_dict:
            # link_dict[link] = get_transform(link, tf_listener)
            # link_dict[link] = get_transform(link, tf_buffer)

            try:
                transform_stamped = tf_buffer.lookup_transform(BASE, link, rospy.Time(0))
                translation_rotation = tf_conversions.toTf(tf2_kdl.transform_to_kdl(transform_stamped))
            except Exception as e2:
                rospy.logerr("get_transform::\n " +
                             "Failed to find transform from %s to %s" % (
                             BASE, link,))
            link_dict[link] = translation_rotation


        if link_dict:
            pub_string = message_builder(link_dict)
            pub.publish(pub_string)

        rate.sleep()

if __name__ == '__main__':
    main()




#old version that is potentially useful

# #!/usr/bin/env python

# import rospy
# from std_msgs.msg import String
# import tf

# BASE = "map"


# def message_builder(link_dict):
#     """
#     Builds the message to send on the ROS network to the VR computer
#     Returns:
#         msg (string): the message
#     """

#     msg = ""
#     for k, v in link_dict.iteritems():
#         trans, rot = v
#         trans = [float("{0:.3f}".format(n)) for n in trans]
#         rot = [float("{0:.3f}".format(n)) for n in rot]
#         msg += '{}:{}^{};'.format(k, trans, rot)
#     # remove spaces to save on space
#     return msg.replace(' ', '')


# def get_transform(link, tf_listener):
#     """
#     update the link_dict with the position and rotation (transform) of the link relative to the base of the robot
#     Params:
#         link (string): name of the link to get transform of
#     """
#     try:
#         t = tf_listener.getLatestCommonTime(BASE, link)
#         (trans, rot) = tf_listener.lookupTransform(BASE, link, t)
#     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
#         return
#     return trans, rot


# def main():
#     # initialize the ROS node
#     rospy.init_node("unityNode", anonymous=False)

#     # set up the publisher
#     pub = rospy.Publisher("ros_unity", String, queue_size=0)
#     # set up the tf listener
#     tf_listener = tf.TransformListener()

#     # create a rate to sleep after every loop
#     rate = rospy.Rate(60)

#     trans = (0, 0, 0)
#     rot = (0, 0, 0, 0)

#     # sleep to give time for publishers and listener to initilize
#     rospy.Rate(1).sleep()

#     # dictionary to store the position and rotation of each link in the robot
#     link_dict = dict()

#     # initialize keys in linkDict
#     for link in tf_listener.getFrameStrings():
#         if 'reference' not in link:
#             link_dict[link] = (trans, rot)

#     # main loop. Updates the values for each link in link_dict and publish the values
#     while not rospy.is_shutdown():
#         for link in link_dict:
#             link_dict[link] = get_transform(link, tf_listener)

#         pub_string = message_builder(link_dict)
#         pub.publish(pub_string)
#         rate.sleep()

# if __name__ == '__main__':
#     main()