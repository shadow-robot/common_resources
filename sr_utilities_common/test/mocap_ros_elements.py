#!/usr/bin/env python3

# Copyright 2020, 2022 Shadow Robot Company Ltd.
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
# pylint: disable=W0613

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty


def empty_callback(req):
    pass


if __name__ == "__main__":
    rospy.init_node("mocap_ros_integration_test_publisher")
    topic_mocap_pub = rospy.Publisher("test_mocap_topic", String, queue_size=10)
    service_mocap = rospy.Service("test_mocap_service", Empty, empty_callback)
    rospy.set_param('test_mocap_param', True)

    r = rospy.Rate(10)

    test_string = String()
    test_string.data = "test"
    while not rospy.is_shutdown():
        topic_mocap_pub.publish(test_string)
        r.sleep()
    rospy.spin()
