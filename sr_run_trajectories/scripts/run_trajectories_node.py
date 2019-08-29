#!/usr/bin/env python
#
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
import rospkg
from sr_run_trajectories.run_trajectories import SrRunTrajectories

if __name__ == "__main__":
    rospy.init_node('run_trajectories_node')
    trajectories_file_path = rospkg.RosPack().get_path('sr_run_trajectories') + '/config/example_trajectories.yaml'

    srt = SrRunTrajectories(trajectories_file_path)
    srt.run_trajectory('arm', 'test_trajectory')
    srt.run_trajectory('hand', 'test_trajectory')
    srt.run_trajectory('arm_and_hand', 'test_trajectory')
