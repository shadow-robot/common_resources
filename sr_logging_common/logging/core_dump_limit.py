#!/usr/bin/env python3

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

import os
import time
import rospy


def get_folder_size(folder):
    if os.path.isdir(folder):
        total_size = os.path.getsize(folder)
        for item in os.listdir(folder):
            itempath = os.path.join(folder, item)
            if os.path.isfile(itempath):
                total_size += os.path.getsize(itempath)
            elif os.path.isdir(itempath):
                total_size += get_folder_size(itempath)
    else:
        total_size = 0
    return total_size


if __name__ == '__main__':
    rospy.init_node('core_dump_limit', anonymous=True)
    desired_size = rospy.get_param('~desired_folder_size', 1024000000)
    path = rospy.get_param('~core_dump_path', '/home/user/.ros/log/core_dumps')
    while not rospy.is_shutdown():
        if get_folder_size(path) > desired_size:
            oldest = min(os.listdir(path), key=lambda f: os.path.getctime("{}/{}".format(path, f)))
            rospy.loginfo("Core dump size greater than limit. Removing oldest file: " + oldest)
            os.remove(path + '/' + oldest)
        time.sleep(5)
