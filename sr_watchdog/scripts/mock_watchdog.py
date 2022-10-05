#!/usr/bin/python3

# Copyright 2019, 2022 Shadow Robot Company Ltd.
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

from __future__ import absolute_import
import rospy
from sr_watchdog.watchdog import SrWatchdog, SrWatchdogChecks


class MockChecksClass(SrWatchdogChecks):
    def __init__(self):
        SrWatchdogChecks.__init__(self, "default")
        self.tmp = [0, 0, 0]

    @SrWatchdogChecks.watchdog_warning_check
    def mock_check_robot_clear_from_collision(self):
        rospy.sleep(6)
        if self.tmp[0] != 1:
            self.tmp[0] = 1
            return False
        self.tmp[0] = 0
        return True

    @SrWatchdogChecks.watchdog_error_check
    def mock_check_if_arm_running(self):
        rospy.sleep(10)
        if self.tmp[1] != 0:
            self.tmp[1] = 0
            return False
        self.tmp[1] = 1
        return True

    @SrWatchdogChecks.watchdog_error_check
    def mock_check_if_hand_running(self):
        rospy.sleep(3)
        self.tmp[2] += 1
        if self.tmp[2] > 5 and self.tmp[2] < 10:
            return (False, "Hand not running!")
        if self.tmp[2] == 36:
            self.tmp[2] = 0
            return True
        return True


if __name__ == '__main__':
    rospy.init_node('mock_sr_teleop_watchdog')

    mock_class = MockChecksClass()
    mock_watchdog = SrWatchdog("mock system", [mock_class])
    mock_watchdog.run()
    rospy.spin()
