#!/usr/bin/env python3

# Copyright 2019, 2022-2023 Shadow Robot Company Ltd.
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

import subprocess
import select
import os
import rospy
from sr_msgs_common.msg import journal_log


if __name__ == "__main__":
    rospy.init_node('journalctl_pub', anonymous=True)
    pub = rospy.Publisher('journalctl_log', journal_log, queue_size=10)

    if os.path.isdir("/host_run"):
        args = ['journalctl', '--directory', '/host_run/log/journal/', '--lines', '0', '--follow']
    else:
        args = ['journalctl', '--lines', '0', '--follow']
    with subprocess.Popen(args, stdout=subprocess.PIPE) as process:
        polling_object = select.poll()
        polling_object.register(process.stdout)

        while not rospy.is_shutdown():
            if polling_object.poll(100):
                journal_log_line = process.stdout.readline()
                if journal_log_line:
                    journal_msg = journal_log()
                    journal_msg.data = journal_log_line.strip()
                    journal_msg.timestamp = rospy.get_rostime()
                    pub.publish(journal_msg)
