#!/usr/bin/env python3
#
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

import rospy
import yaml
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder
from moveit_commander.exception import MoveItCommanderException


def parse_trajectories_dict(trajectories_dict, joints_order):
    parsed_trajectories_dict = {}
    for traj_name, traj_waypoint in trajectories_dict.items():
        parsed_trajectory = []
        for waypoint in traj_waypoint:
            joint_angles_dict = dict(list(zip(joints_order, waypoint['joint_angles'])))
            interpolate_time = waypoint['interpolate_time']
            parsed_trajectory.append({'joint_angles': joint_angles_dict, 'interpolate_time': interpolate_time})
        parsed_trajectories_dict[traj_name] = parsed_trajectory
    return parsed_trajectories_dict


class SrRunTrajectories:
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

        if hand:
            self.hand_prefix = None
            self.hand_side = None
            self.get_hand_prefix()
            self.get_hand_side()

        self._init_moveit_commanders()
        self._parse_all_trajectories(trajectories_file_path)

    def _init_moveit_commanders(self):
        if self.arm:
            self._arm_commander = SrArmCommander(name='right_arm_and_wrist', set_ground=False)
            if self._arm_commander is None:
                raise MoveItCommanderException('Failed to initialise arm commander!')
            self._arm_commander.set_max_velocity_scaling_factor(0.3)
        if self.hand:
            self._hand_commander = SrHandCommander(name='{}_hand'.format(self.hand_side))
            if self._hand_commander is None:
                raise MoveItCommanderException('Failed to initialise hand commander!')
        if self.arm_and_hand:
            self._arm_and_hand_commander = SrArmCommander(name='{}_arm_and_hand'.format(self.hand_side),
                                                          set_ground=False)
            if self._arm_and_hand_commander is None:
                raise MoveItCommanderException('Failed to initialise arm and hand commander!')
            self._arm_and_hand_commander.set_max_velocity_scaling_factor(0.3)

    def _parse_all_trajectories(self, trajectories_file_path):
        with open(trajectories_file_path, 'r', encoding='UTF-8') as stream:
            trajectories_info = yaml.safe_load(stream)

        if self.arm:
            arm_joints_order = trajectories_info['arm_joints_order']
            arm_trajectories = trajectories_info['arm_trajectories']
            self._arm_trajectories = parse_trajectories_dict(arm_trajectories, arm_joints_order)

        if self.hand:
            hand_joints_order = [self.hand_prefix + joint for joint in trajectories_info['hand_joints_order']]
            hand_trajectories = trajectories_info['hand_trajectories']
            self._hand_trajectories = parse_trajectories_dict(hand_trajectories, hand_joints_order)

        if self.arm_and_hand:
            arm_and_hand_joints_order = trajectories_info['arm_joints_order'] + trajectories_info['hand_joints_order']
            arm_and_hand_trajectories = trajectories_info['arm_and_hand_trajectories']
            self._arm_and_hand_trajectories = parse_trajectories_dict(arm_and_hand_trajectories,
                                                                      arm_and_hand_joints_order)

    def run_trajectory(self, configuration, trajectory_name):
        if configuration == 'arm':
            self._arm_commander.run_named_trajectory(self._arm_trajectories[trajectory_name])
        elif configuration == 'hand':
            self._hand_commander.run_named_trajectory(self._hand_trajectories[trajectory_name])
        elif configuration == 'arm_and_hand':
            self._arm_and_hand_commander.run_named_trajectory(self._arm_and_hand_trajectories[trajectory_name])
        else:
            rospy.logerr("Unknown configuration!")

    def get_hand_prefix(self):
        hand_finder = HandFinder()
        joint_prefix_dict = hand_finder.get_hand_parameters().joint_prefix
        if len(joint_prefix_dict) > 1:
            raise ValueError("More then one hand connected, unsupported!")
        values_view = list(joint_prefix_dict.values())
        value_iterator = iter(values_view)
        self.hand_prefix = next(value_iterator)

    def get_hand_side(self):
        if self.hand_prefix == 'rh_':
            self.hand_side = 'right'
        elif self.hand_prefix == 'lh_':
            self.hand_side = 'left'
        else:
            raise ValueError("Unknown hand prefix!")

    def get_hand_trajectories(self):
        return self._hand_trajectories
