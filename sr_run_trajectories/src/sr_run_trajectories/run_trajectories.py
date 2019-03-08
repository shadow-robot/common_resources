#!/usr/bin/env python
#
# Copyright (C) 2017 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.


from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
import rospy

class SrRunTrajectories(object):
    def __init__(self):
        self.arm_trajectories = {}
        self.hand_trajectories = {}
        self.arm_and_hand_trajectories = {}

        self._init_moveit_commanders()
        self._parse_all_trajectories()

    def _init_moveit_commanders(self):
        self.arm_commander = SrArmCommander(name='right_arm_and_wrist', set_ground=False)
        self.hand_commander = SrHandCommander(name='right_hand')
        self.arm_and_hand_commander = SrArmCommander(name='right_arm_and_hand', set_ground=False)
        if self.arm_commander is None or self.hand_commander is None or self.arm_and_hand_commander is None:
            raise MoveItCommanderException('Failed to initialise robot commander!')
        self.arm_commander.set_max_velocity_scaling_factor(0.3)
        self.arm_and_hand_commander.set_max_velocity_scaling_factor(0.1)

    def _parse_all_trajectories(self):
        arm_joints_order = rospy.get_param("/arm_joints_order")
        hand_joints_order = rospy.get_param("/hand_joints_order")
        arm_and_hand_joints_order = rospy.get_param("/arm_and_hand_joints_order")

        arm_trajectories = rospy.get_param("/arm_trajectories", {})
        hand_trajectories = rospy.get_param("/hand_trajectories", {})
        arm_and_hand_trajectories = rospy.get_param("/arm_and_hand_trajectories", {})

        self.arm_trajectories = self._parse_trajectories_dict(arm_trajectories, arm_joints_order)
        self.hand_trajectories = self._parse_trajectories_dict(hand_trajectories, hand_joints_order)
        self.arm_and_hand_trajectories = self._parse_trajectories_dict(arm_and_hand_trajectories, arm_and_hand_joints_order)

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
            self.arm_commander.run_named_trajectory(self.arm_trajectories[trajectory_name])
        elif configuration is 'hand':
            self.hand_commander.run_named_trajectory(self.hand_trajectories[trajectory_name])
        elif configuration is 'arm_and_hand':
            self.arm_and_hand_commander.run_named_trajectory(self.arm_and_hand_trajectories[trajectory_name])
        else:
            rospy.logerr("Unknown configuration!")
