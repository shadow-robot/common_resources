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

# WHEN TO USE:
# Use this script when you have a rosbag and wand to play the trajectory data to a hand
# or when you are using a right glove (i.e. cyberglove) and you are using a left shadow hand.
#
# HOW TO USE:
# To use the trajectory republisher, select if you are using a left hand.
# Then, select which joints you want to move (don't change the prefix).
# Run your publisher and then start your rosbag by remapping your topic for instance
# by adding the following line in the end of your rosbag command
#
# /rh_trajectory_controller/command:=/rh_trajectory_controller/command_remapped
#
# Your hand should be start moving.

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class RePubTrajectoryAsJointStates(object):
    def __init__(self,
                 joints_to_move,
                 left_hand=False,
                 remapped_trajectory_sub_topic="/rh_trajectory_controller/command",
                 joint_states_pub_topic="/remapped_joint_states"):
        self._bag_tf_sub = rospy.Subscriber(remapped_trajectory_sub_topic,
                                            JointTrajectory,
                                            self._traj_cb,
                                            tcp_nodelay=True)
        self._joint_states_pub = rospy.Publisher(joint_states_pub_topic,
                                         JointState,
                                         queue_size=10)
        self._joints_to_move = joints_to_move
        self._right_hand_prefix = "rh_"
        self._left_hand_prefix = "lh_"
        self._left_hand = left_hand

    def _traj_cb(self, data):
        new_traj = JointState()
        new_traj.header = data.header
        for index, joint_name in enumerate(data.joint_names):
            if joint_name in self._joints_to_move:
                if self._left_hand:
                    joint_name = joint_name.replace(self._right_hand_prefix, self._left_hand_prefix)
                new_traj.name.append(joint_name)
                new_traj.position.append(data.points[0].positions[index])
        new_traj.header.stamp = rospy.Time.now()
        self._joint_states_pub.publish(new_traj)


if __name__ == "__main__":
    rospy.init_node("republish_trajectory")

    # Select if left hand is used
    left_hand = False

    joint_states_pub_topic = "/remapped_joint_states"

    joints_to_move = ["rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4",
                      "rh_MFJ1", "rh_MFJ2", "rh_MFJ3", "rh_MFJ4",
                      "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4",
                      "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5",
                      "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5",
                      "rh_WRJ1", "rh_WRJ2"]

    pub_traj = RePubTrajectoryAsJointStates(joints_to_move, left_hand=left_hand, joint_states_pub_topic=joint_states_pub_topic)
    rospy.spin()
