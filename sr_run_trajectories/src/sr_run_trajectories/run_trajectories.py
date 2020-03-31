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
import yaml
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander


class SrRunTrajectories(object):
    def __init__(self, trajectories_file_path, arm=True, hand=True):
        self.arm = arm
        self.hand = hand
        self.arm_and_hand = arm and hand

        self._arm_trajectories = {}
        self._hand_trajectories = {}
        self._arm_and_hand_trajectories = {}

        self._arm_commander = None
        self._hand_commander = None
        self._arm_and_hand_commander = None

        self._init_moveit_commanders()
        self._parse_all_trajectories(trajectories_file_path)

    def _init_moveit_commanders(self):
        if self.arm:
            self._arm_commander = SrArmCommander(name='right_arm_and_wrist', set_ground=False)
            if self._arm_commander is None:
                raise MoveItCommanderException('Failed to initialise arm commander!')
            self._arm_commander.set_max_velocity_scaling_factor(0.3)
        if self.hand:
            self._hand_commander = SrHandCommander(name='right_hand')
            if self._hand_commander is None:
                raise MoveItCommanderException('Failed to initialise hand commander!')
        if self.arm_and_hand:
            self._arm_and_hand_commander = SrArmCommander(name='right_arm_and_hand', set_ground=False)
            if self._arm_and_hand_commander is None:
                raise MoveItCommanderException('Failed to initialise arm and hand commander!')
            self._arm_and_hand_commander.set_max_velocity_scaling_factor(0.3)

    def _parse_all_trajectories(self, trajectories_file_path):
        with open(trajectories_file_path, 'r') as stream:
            trajectories_info = yaml.load(stream)

        if self.arm:
            arm_joints_order = trajectories_info['arm_joints_order']
            arm_trajectories = trajectories_info['arm_trajectories']
            self._arm_trajectories = self._parse_trajectories_dict(arm_trajectories, arm_joints_order)

        if self.hand:
            hand_joints_order = trajectories_info['hand_joints_order']
            hand_trajectories = trajectories_info['hand_trajectories']
            self._hand_trajectories = self._parse_trajectories_dict(hand_trajectories, hand_joints_order)

        if self.arm_and_hand:
            arm_and_hand_joints_order = trajectories_info['arm_and_hand_joints_order']
            arm_and_hand_trajectories = trajectories_info['arm_and_hand_trajectories']
            self._arm_and_hand_trajectories = self._parse_trajectories_dict(arm_and_hand_trajectories,
                                                                            arm_and_hand_joints_order)

    def _parse_trajectories_dict(self, trajectories_dict, joints_order):
        parsed_trajectories_dict = {}
        for traj_name, traj_waypoint in trajectories_dict.iteritems():
            parsed_trajectory = []
            for waypoint in traj_waypoint:
                joint_angles_dict = dict(zip(joints_order, waypoint['joint_angles']))
                interpolate_time = waypoint['interpolate_time']
                parsed_trajectory.append({'joint_angles': joint_angles_dict,
                                          'interpolate_time': interpolate_time})
            parsed_trajectories_dict[traj_name] = parsed_trajectory
        return parsed_trajectories_dict

    def run_trajectory(self, configuration, trajectory_name):
        if configuration is 'arm':
            self._arm_commander.run_named_trajectory(self._arm_trajectories[trajectory_name])
        elif configuration is 'hand':
            self._hand_commander.run_named_trajectory(self._hand_trajectories[trajectory_name])
        elif configuration is 'arm_and_hand':
            self._arm_and_hand_commander.run_named_trajectory(self._arm_and_hand_trajectories[trajectory_name])
        else:
            rospy.logerr("Unknown configuration!")
