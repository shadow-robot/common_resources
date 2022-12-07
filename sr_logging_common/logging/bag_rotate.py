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

from os import listdir, remove, rename
from os.path import exists
import subprocess
import rospy


def reindex_bag(active_bag):
    reindexing = subprocess.run(["rosbag", "reindex", active_bag], capture_output=True, text=True, check=False)
    fixed_bag = f"{active_bag[:-len('.active')]}"

    if reindexing.returncode == 0:
        backup_file = f"{fixed_bag}.orig.active"
        if exists(backup_file):
            remove(backup_file)
        rename(active_bag, fixed_bag)


class SrBagRotate:
    def __init__(self, path, desired_bag_number, prefix):
        self._path = path
        self._prefix = prefix
        self._desired_prefix_parts = set(self._prefix.split("_"))
        self._desired_bag_number = desired_bag_number
        self.run()

    def remove_old_bags(self):
        bags_to_remove = self.get_suffixed_bags(".bag")
        bags_to_remove.extend(self.get_suffixed_bags(".bag.orig.active"))
        bags_to_remove.sort()

        for i, bag in enumerate(bags_to_remove):
            if i < len(bags_to_remove)-self._desired_bag_number - 1:
                remove(bag)

    def get_suffixed_bags(self, suffix):
        all_files = listdir(self._path)
        all_bags = [file for file in all_files if file.endswith(suffix)]
        suffixed_bags = []
        for bag in all_bags:
            actual_prefix_parts = set([t for t in bag.split("_") if t.isalpha()])
            if self._desired_prefix_parts == actual_prefix_parts:
                suffixed_bags.append(f"{self._path}/{bag}")
        return suffixed_bags

    def run(self):
        while not rospy.is_shutdown():
            active_bags = self.get_suffixed_bags(".bag.active")
            for bag in active_bags:
                reindex_bag(bag)
            self.remove_old_bags()
            rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('bag_rotate', anonymous=True)

    desired_bag_number_param = rospy.get_param('~bag_files_num')
    path_param = rospy.get_param('~bag_files_path', '/home/user/.ros/log')
    file_name_prefix_param = rospy.get_param('~bag_files_prefix', '')

    SrBagRotate(path_param, desired_bag_number_param, file_name_prefix_param)
