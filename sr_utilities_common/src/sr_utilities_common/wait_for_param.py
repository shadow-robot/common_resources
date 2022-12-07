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

import time
import rospy

CONST_TIME_BEFORE_INFO = 60


def wait_for_param(param_name, timeout_in_secs=0):
    start_time = time.time()
    if timeout_in_secs <= 0:

        while not rospy.is_shutdown():
            if rospy.has_param(param_name):
                return True

            if time.time() - start_time >= CONST_TIME_BEFORE_INFO:
                rospy.loginfo("Still waiting for parameter: {}".format(param_name))
                start_time = time.time()

            rospy.sleep(0.1)
        return False

    while time.time() - start_time < timeout_in_secs:
        if rospy.has_param(param_name):
            return True
        rospy.sleep(0.1)
    return False
