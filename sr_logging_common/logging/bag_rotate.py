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
from os import listdir, remove
from os.path import getctime, join
import time
import rospy
import subprocess


def remover(desired_bag_number, path):
    CONST_MAX_ACTIVE_BAGS_ALLOWED = desired_bag_number//2

    while not rospy.is_shutdown():
        bag_files = [bagfile for bagfile in listdir(path) if bagfile.endswith('.bag')]
        active_bag_files = [bagfile for bagfile in listdir(path) if bagfile.endswith('.bag.active')]

        sorted_bag_files = sorted(bag_files, key=lambda x: getctime(join(path, x)))
        sorted_active_bag_files = sorted(active_bag_files, key=lambda x: getctime(join(path, x)))

        while len(sorted_bag_files) > desired_bag_number:
            remove(join(path, sorted_bag_files[0]))
            sorted_bag_files.pop(0)

        while len(sorted_active_bag_files) > CONST_MAX_ACTIVE_BAGS_ALLOWED:
            remove(join(path, sorted_active_bag_files[0]))
            sorted_active_bag_files.pop(0)

        time.sleep(1)


def gather_and_fix_all_active_rosbag_files(path):
    active_rosbags = [bagfile for bagfile in listdir(path) if bagfile.endswith('.bag.active')]
    rospy.loginfo("Fixing unfinished rosbag files")
    for bag_file in active_rosbags:
        file_name = bag_file.split(".")[0] + ".bag"
        subprocess.run(["rosbag", "reindex", bag_file])
        subprocess.run(["rosbag", "fix", bag_file, file_name])
        remove(file_name + ".active")
        remove(file_name + ".orig.active")

    if len(active_rosbags) > 0:
        string = "Fixed the rosbag files:"
        for filename in active_rosbags:
            string += f"  {filename}"
        rospy.loginfo(string)


if __name__ == '__main__':
    rospy.init_node('bag_rotate', anonymous=True)
    desired_bag_number = rospy.get_param('~bag_files_num')
    path = rospy.get_param('~bag_files_path')
    gather_and_fix_all_active_rosbag_files(path)
    remover(desired_bag_number, path)
