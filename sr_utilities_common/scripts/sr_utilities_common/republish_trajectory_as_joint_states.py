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

# WHEN TO USE:
# Use this script when you want to simulate (visualize) a robot that executes the trajectory commands
# it receives. Only the first trajectory point will be considered. This is useful for teleoperation,
# where we send a continuous stream of single point trajectories.
#
# HOW TO USE:
# To use the trajectory republisher, select if you are using a left hand.
# Then, select which joints you want to move (don't change the prefix).
# Run your publisher and remap topic names if necessary.
#

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class RePubTrajectoryAsJointStates:
    def __init__(self,
                 joints_to_move,
                 robot_side="",
                 trajectory_cmd_topic="/rh_trajectory_controller/command",
                 joint_states_pub_topic="/remapped_joint_states"):
        self._bag_tf_sub = rospy.Subscriber(trajectory_cmd_topic,
                                            JointTrajectory,
                                            self._traj_cb,
                                            tcp_nodelay=True)
        self._joint_states_pub = rospy.Publisher(joint_states_pub_topic,
                                                 JointState,
                                                 queue_size=10)
        self._joints_to_move = joints_to_move
        self._right_hand_prefix = "rh_"
        self._left_hand_prefix = "lh_"
        self._right_arm_prefix = "ra_"
        self._left_arm_prefix = "la_"
        self._robot_side = robot_side

    def _traj_cb(self, data):
        new_state = JointState()
        new_state.header = data.header
        for index, joint_name in enumerate(data.joint_names):
            if (joint_name in self._joints_to_move) or (not self._joints_to_move):
                if self._robot_side == "left":
                    joint_name = joint_name.replace(self._right_hand_prefix, self._left_hand_prefix)
                    joint_name = joint_name.replace(self._right_arm_prefix, self._left_arm_prefix)
                elif self._robot_side == "right":
                    joint_name = joint_name.replace(self._left_hand_prefix, self._right_hand_prefix)
                    joint_name = joint_name.replace(self._left_arm_prefix, self._right_arm_prefix)
                new_state.name.append(joint_name)
                new_state.position.append(data.points[0].positions[index])
        new_state.header.stamp = rospy.Time.now()
        self._joint_states_pub.publish(new_state)


if __name__ == "__main__":
    rospy.init_node("republish_trajectory")

    # Select we want to translate a right side robot command to control a left side robot or vice versa
    robot_side_param = rospy.get_param("~robot_side", "")
    joint_states_pub_topic_param = rospy.get_param("~remapped_joint_states_topic_name", "/remapped_joint_states")
    trajectory_cmd_topic_param = rospy.get_param("~trajectory_cmd_topic_name", "/rh_trajectory_controller/command")
    joints_to_move_param = rospy.get_param("~joints_to_move", ["rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4",
                                                               "rh_MFJ1", "rh_MFJ2", "rh_MFJ3", "rh_MFJ4",
                                                               "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4",
                                                               "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5",
                                                               "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5",
                                                               "rh_WRJ1", "rh_WRJ2"])

    pub_traj = RePubTrajectoryAsJointStates(joints_to_move_param, robot_side=robot_side_param,
                                            trajectory_cmd_topic=trajectory_cmd_topic_param,
                                            joint_states_pub_topic=joint_states_pub_topic_param)
    rospy.spin()
