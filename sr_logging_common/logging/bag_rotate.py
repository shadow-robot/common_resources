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


class SrBagRotate:
    def __init__(self, path, desired_bag_number, prefix):
        self._path = path
        self._prefix = prefix
        self._desired_prefix_parts = set(self._prefix.split("_"))
        self._desired_bag_number = desired_bag_number
        self.remove_old_backups()
        self.run()

    def remove_old_backups(self):
        for file in self.get_suffixed_bags(".bag.orig.active"):
            remove(self._path + "/" + file)

    def get_suffixed_bags(self, suffix):
        all_files = listdir(self._path)
        all_active_bags = [file for file in all_files if file.endswith(suffix)]
        prefixed_active_bags = []
        for active_bag in all_active_bags:
            actual_prefix_parts = set([t for t in active_bag.split("_") if t.isalpha()])
            if self._desired_prefix_parts == actual_prefix_parts:
                prefixed_active_bags.append(active_bag)
        return prefixed_active_bags

    def reindex_bag(self, active_bag_filename):
        process = subprocess.run(["rosbag", "reindex", active_bag_filename], capture_output=True, text=True)
        if process.returncode == 0:
            active_bag_filename = self._path + "/" + active_bag_filename.split(".")[0]
            if exists(active_bag_filename + ".bag.active"):
                remove(active_bag_filename + ".bag.active")
            if exists(active_bag_filename + ".bag.orig.active"):
                remove(active_bag_filename + ".bag.orig.active")
            return True

        rospy.logerr(f"\nReindexing failed: \n{process.stdout}\n{process.stderr}")
        return False

    def run(self):
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():

            active_bags = self.get_suffixed_bags(".bag.active")
            for bag in active_bags:
                self.reindex_bag(bag)

            sorted_bag_files = sorted(self.get_suffixed_bags(".bag"))
            if len(sorted_bag_files) > self._desired_bag_number:
                remove(self._path + "/" + sorted_bag_files[0])

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('bag_rotate', anonymous=True)
    desired_bag_number = rospy.get_param('~bag_files_num', 6)
    path = rospy.get_param('~bag_files_path', '/home/user/.ros/log')
    file_name_prefix = rospy.get_param('~bag_files_prefix', '')
    SrBagRotate(path, desired_bag_number, "sr_hand")
