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

import shutil
import rospy
import logging_utils as utils


if __name__ == '__main__':
    rospy.init_node('check_log_size')

    # -----------------------------------------Definition of parameters-------------------------------------------------
    # non_log_headroom:     Additional disk space allocated for non-log file growth during a driver run.
    # allocated_log_space:  Minimum disk space allocated for roslogs. Logs are allowed to grow beyond this size.
    # log_growth_headroom:  Additional disk space allocated for log growth during a run. This is added to the current
    #                       log size or allocated_log_space whichever is greater.
    # log_size:             Current size of logs.
    # ------------------------------------------------------------------------------------------------------------------

    non_log_headroom = rospy.get_param('~additional_required_disk_space', 10) * utils.GIGABYTE
    allocated_log_space = rospy.get_param('~allocated_log_space', 10) * utils.GIGABYTE
    log_growth_headroom = rospy.get_param('~log_growth_headroom', 5) * utils.GIGABYTE

    total_disk_space, _, free_disk_space = shutil.disk_usage(utils.LOG_PATH)
    logs_size = utils.get_directory_size(utils.LOG_PATH)

    if free_disk_space < log_growth_headroom + max(logs_size, allocated_log_space) + non_log_headroom:
        required_disk_space = log_growth_headroom + max(logs_size, allocated_log_space) + non_log_headroom
        rospy.logfatal("Not enough free disk space to safely store roslogs:\n" +
                       f"\tFree Disk Space: {free_disk_space / utils.GIGABYTE:.3f} of"
                       f" {total_disk_space / utils.GIGABYTE:.3f} GB free\n" +
                       f"\tRequired Free Disk Space: {required_disk_space / utils.GIGABYTE:.3f} GB\n" +
                       f"\t\tNon-log Headroom: {non_log_headroom / utils.GIGABYTE:.3f} GB\n" +
                       f"\t\tAllocated Log Space: {allocated_log_space  / utils.GIGABYTE:.3f} GB\n" +
                       f"\t\tLog Growth Headroom: {log_growth_headroom  / utils.GIGABYTE:.3f} GB\n" +
                       f"\t\tCurrent Log Size: {logs_size / utils.GIGABYTE:.3f} GB")

        if logs_size > allocated_log_space:
            rospy.logwarn("Current size of logs exceeds allocated log space. Consider running clean_logs.py in" +
                          " sr_logging_common to free up some space. Command: rosrun sr_logging_common clean_logs.py")
        else:
            rospy.logwarn(f"Current size of logs is {logs_size / utils.GIGABYTE:.3f} GB."
                          f" {allocated_log_space  / utils.GIGABYTE:.3f} GB are allocated for logs plus an additional" +
                          f" {log_growth_headroom  / utils.GIGABYTE:.3f} GB for log growth during a run.")
            rospy.logwarn("Consider clearing up some space elsewhere on the disk.")

        rospy.signal_shutdown("Shutting down all nodes to prevent further log generation.")

    rospy.spin()
