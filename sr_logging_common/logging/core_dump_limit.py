#!/usr/bin/env python3

# Copyright 2019, 2022, 2024-2025 Shadow Robot Company Ltd.
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
import sr_logging_common.logging_utils as utils


if __name__ == '__main__':
    rospy.init_node('core_dump_limit', anonymous=True)
    desired_size = rospy.get_param('~desired_folder_size', utils.GIGABYTE)
    path = rospy.get_param('~core_dump_path', utils.CORE_DUMPS_PATH)
    while not rospy.is_shutdown():
        try:
            directory_size = utils.get_directory_size(path)
        except (FileNotFoundError, NotADirectoryError):
            directory_size = 0
        if directory_size > desired_size:
            oldest, _ = utils.get_oldest_item_in_directory(path)
            rospy.loginfo("Core dump size greater than limit. Removing oldest file: " + oldest)
            os.remove(os.path.join(path, oldest))
        time.sleep(5)
