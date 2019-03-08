#!/usr/bin/env python
#
# Copyright (C) 2017 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import os
import rospy
from sensor_msgs.msg import JointState


if __name__ == "__main__":
    rospy.init_node('print_joint_states')

    while not rospy.is_shutdown():
        raw_input("Press [RETURN] to get joint states...\n")
        os.system('clear')
        joint_states_msg = rospy.wait_for_message("/joint_states", JointState)

        print "Arm joints:\n"
        print [round(x, 3) for x in list(joint_states_msg.position[0:6])]
        print "\n"

        print "Hand joints:\n"
        print [round(x, 3) for x in list(joint_states_msg.position[6:])]
        print "\n"

        print "Arm and hand joints:\n"
        print [round(x, 3) for x in list(joint_states_msg.position)]
        print "\n"