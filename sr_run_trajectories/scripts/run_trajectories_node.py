#!/usr/bin/env python
#
# Copyright (C) 2017 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
from sr_run_trajectories.run_trajectories import SrRunTrajectories

if __name__ == "__main__":
    rospy.init_node('run_trajectories_node')

    srt = SrRunTrajectories()
    srt.run_trajectory('arm', 'test_trajectory')
    srt.run_trajectory('hand', 'test_trajectory')
    srt.run_trajectory('arm_and_hand', 'test_trajectory')
