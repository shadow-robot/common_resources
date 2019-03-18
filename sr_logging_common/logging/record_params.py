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

import rospy
import datetime

rospy.init_node("param_dump", anonymous=True)

run_time = datetime.datetime.fromtimestamp(rospy.get_rostime().secs)

name_string = "run_params_%04d-%02d-%02d-%02d-%02d-%02d" % (
    run_time.year, run_time.month, run_time.day, run_time.hour, run_time.minute, run_time.second)

rospy.sleep(10)

param_tree = rospy.get_param("/")

output = open(rospy.get_param("~log_directory", ".") + "/" + name_string, 'w')

output.write(str(param_tree))
