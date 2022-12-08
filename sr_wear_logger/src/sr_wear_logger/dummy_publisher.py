#!/usr/bin/env python3
#
# Copyright 2021, 2022 Shadow Robot Company Ltd.
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

import random
import rospy
from sensor_msgs.msg import JointState


class DummyPublisher:
    def __init__(self):
        pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        msg = JointState()

        msg.name = ['ra_elbow_joint', 'ra_shoulder_lift_joint', 'ra_shoulder_pan_joint', 'ra_wrist_1_joint',
                    'ra_wrist_2_joint', 'ra_wrist_3_joint', 'rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_LFJ1',
                    'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5', 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4', 'rh_RFJ1',
                    'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4', 'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5', 'rh_WRJ1',
                    'rh_WRJ2']

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            msg.position = len(msg.name) * [random.uniform(0, 0.04)]
            pub.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('dummy_publisher_node')
    server = DummyPublisher()
    rospy.spin()
