#!/usr/bin/env python
#
# Copyright (C) 2021 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import actionlib
from sensor_msgs.msg import JointState


class DummyPublisher(object):
    def __init__(self):
        pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        msg = JointState()
        msg.name = ['ra_elbow_joint', 'ra_shoulder_lift_joint', 'ra_shoulder_pan_joint', 'ra_wrist_1_joint',
                    'ra_wrist_2_joint', 'ra_wrist_3_joint', 'rh_FFJ1', 'rh_FFJ2, rh_FFJ3', 'rh_FFJ4, rh_LFJ1',
                    'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4, rh_LFJ5', 'rh_MFJ1', 'rh_MFJ2, rh_MFJ3', 'rh_MFJ4, rh_RFJ1',
                    'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4, rh_THJ1', 'rh_THJ2', 'rh_THJ3, rh_THJ4', 'rh_THJ5, rh_WRJ1',
                    'rh_WRJ2']
        msg.position = 30 * [0]
        while not rospy.is_shutdown():
            pub.publish(msg)
            rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('dummy_publisher_node')
    server = DummyPublisher()
    rospy.spin()
