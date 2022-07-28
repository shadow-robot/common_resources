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
from os.path import getctime, join, exists
from re import search
import sys
import time
import rospy
import subprocess


def rosbag_starts_and_ends_with(bag_file_name, prefix, suffix):
    if not bag_file_name.endswith(suffix):
        return False

    match = search(r'\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}', bag_file_name)
    bag_prefix = bag_file_name[0:match.span()[0]-1]
    return bag_prefix == prefix


def remover(desired_bag_number, path, file_name_prefix):
    while not rospy.is_shutdown():
        bag_files = [bagfile for bagfile in listdir(path)
                     if rosbag_starts_and_ends_with(bagfile, file_name_prefix, '.bag')]

        sorted_bag_files = sorted(bag_files, key=lambda x: getctime(join(path, x)))

        while len(sorted_bag_files) > desired_bag_number:
            remove(join(path, sorted_bag_files[0]))
            sorted_bag_files.pop(0)

        time.sleep(1)


def gather_and_fix_all_active_rosbag_files(path, file_name_prefix):
    active_rosbags = [join(path, bagfile) for bagfile in listdir(path)
                      if rosbag_starts_and_ends_with(bagfile, file_name_prefix, '.bag.active')]
    for bag_file in active_rosbags:
        file_name = bag_file.split(".active")[0]
        process = subprocess.run(["rosbag", "reindex", bag_file], capture_output=True, text=True)
        if process.returncode != 0:
            rospy.logerr(f"\nCOMMAND REINDEX:\nstdout: {process.stdout}\nstderr: {process.stderr}")
        process = subprocess.run(["rosbag", "fix", bag_file, file_name], capture_output=True, text=True)
        if process.returncode != 0:
            rospy.logerr(f"\nCOMMAND FIX:\nstdout: {process.stdout}\nstderr: {process.stderr}")
        if exists(file_name + ".active"):
            remove(file_name + ".active")
        if exists(file_name + ".orig.active"):
            remove(file_name + ".orig.active")

    if len(active_rosbags) > 0:
        string = "Fixed the rosbag files:"
        for filename in active_rosbags:
            string += f"  {filename}"
        rospy.loginfo(string)


if __name__ == '__main__':
    rospy.init_node('bag_rotate', anonymous=True)
    desired_bag_number = rospy.get_param('~bag_files_num', 6)
    path = rospy.get_param('~bag_files_path', '/home/user/.ros/log')
    file_name_prefix = rospy.get_param('~bag_files_prefix', '')
    if not exists(path):
        rospy.logerr(f"Path {path} cannot be found.")
        sys.exit(1)
    gather_and_fix_all_active_rosbag_files(path, file_name_prefix)
    remover(desired_bag_number, path, file_name_prefix)
