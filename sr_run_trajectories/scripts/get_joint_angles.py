#!/usr/bin/env python
#
# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

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
