#!/usr/bin/env python3

# Copyright 2024 Shadow Robot Company Ltd.
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
import shutil
import rospy
from logging_utils import get_directory_size

GIGABYTE = 1024**3

LOG_PATH = os.path.join(os.path.expanduser('~'), '.ros', 'log')

if __name__ == '__main__':
    rospy.init_node('check_log_size')

    required_disk_space_for_logs = rospy.get_param('~required_disk_space_for_logs', 5 * GIGABYTE)
    min_size_of_logs = rospy.get_param('~min_size_of_logs', 1 * GIGABYTE)

    total_disk_space, _, free_disk_space = shutil.disk_usage(LOG_PATH)

    logs_size = get_directory_size(LOG_PATH)

    if free_disk_space < required_disk_space_for_logs:
        rospy.logfatal("Not enough free disk space to safely store roslogs:\n" +
                     f"\tFree Disk Space: {free_disk_space} of {total_disk_space} bytes free\n" +
                     f"\tRequired Disk Space for roslogs: {required_disk_space_for_logs } bytes\n" +
                     f"\troslog Size: {logs_size} bytes")

        if logs_size > min_size_of_logs:
            rospy.logwarn("Consider running log_cleaner.py to cut the size of the logs down to" +
                          f" {min_size_of_logs} bytes.")
        else:
            rospy.logwarn(f"Up to {min_size_of_logs} bytes of logs should be kept at all times. Consider" +
                          " clearning up some space elsewhere on the disk.")

        rospy.signal_shutdown("Shutting down all nodes to prevent further log generation.")

    rospy.spin()
