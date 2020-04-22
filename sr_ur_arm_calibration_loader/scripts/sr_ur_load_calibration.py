#!/usr/bin/env python
#
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import os
import rospy


class SrUrLoadCalibration():
    def __init__(self):

        while not rospy.is_shutdown():
            try:
                self.run()
            except rospy.ROSInterruptException:
                rospy.loginfo("Shutting down %s", rospy.get_name())

    def run(self):


if __name__ == "__main__":
    rospy.init_node("sr_ur_load_calibration")
    sr_ur_load_calibration = SrUrLoadCalibration()
