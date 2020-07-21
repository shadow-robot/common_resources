#!/usr/bin/env python
#
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

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
    while not rospy.is_shutdown():
        test_string.data = "test"
        topic_mocap_pub.publish(test_string)
        r.sleep()
    rospy.spin()
