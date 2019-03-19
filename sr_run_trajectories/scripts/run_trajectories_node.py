#!/usr/bin/env python
#
# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved.

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
