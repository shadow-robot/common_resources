#!/usr/bin/env python3

# Copyright 2021 Shadow Robot Company Ltd.
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


import math
from sr_robot_msgs.msg import ShadowPST
import rospy
import numpy as np

PST_THRESHOLD = 370.0
PST_SATURATION = 920.0
PST_MAPPING_EXPONENT = 1.5

class Test():

    def __init__(self):
        rospy.Subscriber("/rh/tactile", ShadowPST, self.pst_tactile_cb)

    def pst_tactile_cb(self, data):  

        x = 5 * [None]
        for i, press in enumerate(data.pressure):            
            x[i] = (press - PST_THRESHOLD)/(PST_SATURATION - PST_THRESHOLD)
            x[i] = min(max(x[i], 0), 1)

        rospy.logwarn("{:.2f} {:.2f} {:.2f} {:.2f} {:.2f}".format(x[0],x[1],x[2],x[3],x[4]))

if __name__ == "__main__":

    rospy.init_node('sr_finger_mount')
    Test()
    rospy.spin()
