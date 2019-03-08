#!/usr/bin/env python

# Copyright 2019 Shadow Robot Company Ltd.
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
import rospy
import time


def getFolderSize(folder):
    if os.path.isdir(folder):
        total_size = os.path.getsize(folder)
        for item in os.listdir(folder):
            itempath = os.path.join(folder, item)
            if os.path.isfile(itempath):
                total_size += os.path.getsize(itempath)
            elif os.path.isdir(itempath):
                total_size += getFolderSize(itempath)
    else:
        total_size = 0
    return total_size

if __name__ == '__main__':
    rospy.init_node('core_dump_limit', anonymous=True)
    desired_size = rospy.get_param('~desired_folder_size', 1024000000)
    path = rospy.get_param('~core_dump_path', '/home/user/.ros/log/core_dumps')
    while not rospy.is_shutdown():
        if getFolderSize(path) > desired_size:
            oldest = min(os.listdir(path), key=lambda f: os.path.getctime("{}/{}".format(path, f)))
            rospy.loginfo("Core dump size greater than limit. Removing oldest file: " + oldest)
            os.remove(path + '/' + oldest)
        time.sleep(5)
