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

from __future__ import absolute_import
import os
import time
import rospy
import subprocess


def remover(desired_bag_number, path):
    CONST_MAX_ACTIVE_BAGS_ALLOWED = desired_bag_number//2

    while not rospy.is_shutdown():
        bag_files = [f for f in os.listdir(path) if f.endswith('.bag')]
        active_bag_files = [f for f in os.listdir(path) if f.endswith('.bag.active')]

        sorted_bag_files = sorted(bag_files, key=lambda x: os.path.getctime(os.path.join(path, x)))
        sorted_active_bag_files = sorted(active_bag_files, key=lambda x: os.path.getctime(os.path.join(path, x)))

        while len(sorted_bag_files) > desired_bag_number:
            os.remove(os.path.join(path, sorted_bag_files[0]))
            sorted_bag_files.pop(0)

        while len(sorted_active_bag_files) > CONST_MAX_ACTIVE_BAGS_ALLOWED:
            os.remove(os.path.join(path, sorted_active_bag_files[0]))
            sorted_active_bag_files.pop(0)

        time.sleep(1)


def gather_and_fix_all_active_rosbag_files(path):
    fixed_files = []
    onlyfiles = [os.path.join(path, rosfile) for rosfile in os.path.listdir(path)
                 if os.path.isfile(os.path.join(path, rosfile))]
    for bag_file in onlyfiles:
        if ".active" in bag_file and ".bag" in bag_file:
            file_name = bag_file.split(".")[0] + ".bag"
            subprocess.run(["rosbag", "reindex", bag_file])
            subprocess.run(["rosbag", "fix", bag_file, file_name])
            os.remove(file_name + ".active")
            os.remove(file_name + ".orig.active")
            fixed_files.append(file_name)
    if len(fixed_files) > 0:
        print("Fixed the rosbag files:")
        [print("  " + filename) for filename in fixed_files]


if __name__ == '__main__':
    rospy.init_node('bag_rotate', anonymous=True)
    desired_bag_number = rospy.get_param('~bag_files_num')
    path = rospy.get_param('~bag_files_path')
    gather_and_fix_all_active_rosbag_files(path)
    remover(desired_bag_number, path)
