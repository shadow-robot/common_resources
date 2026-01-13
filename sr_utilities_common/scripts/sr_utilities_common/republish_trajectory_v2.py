#!/usr/bin/env python3

# Copyright 2026 Shadow Robot Company Ltd.
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

from threading import Lock
from functools import partial
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class RePubTrajectory:
    def __init__(self, topics_to_republish, subscribed_subtopic, published_subtopic):
        self._mutex = Lock()
        self._topics_first_msg_timestamp = None  # Timestamp of first msg received amongst all topics to republish
        self._current_time_of_first_msg = None  # Current time when first msg was received

        for topic in topics_to_republish:
            self._traj_pub = rospy.Publisher(topic + published_subtopic,
                                             JointTrajectory,
                                             queue_size=10)
            self._bag_tf_sub = rospy.Subscriber(topic + subscribed_subtopic,
                                                JointTrajectory,
                                                partial(self._bag_traj_cb, self._traj_pub))

    def _bag_traj_cb(self, republisher, data):
        if self._topics_first_msg_timestamp is None:
            with self._mutex:
                self._topics_first_msg_timestamp = data.header.stamp
                self._first_msg_current_time = rospy.Time.now()

        new_traj = JointTrajectory()

        new_traj.header = data.header
        new_traj.header.stamp = data.header.stamp - self._topics_first_msg_timestamp + self._first_msg_current_time
        new_traj.header.stamp += rospy.Time.from_sec(50 / 1000)  # Shift timestamp to 50ms in the future

        new_traj.points = data.points
        new_traj.joint_names = data.joint_names

        republisher.publish(new_traj)


if __name__ == "__main__":
    rospy.init_node("republish_trajectory")

    topics_to_republish = ["/rh_trajectory_controller",
                           "/lh_trajectory_controller",
                           "/ra_trajectory_controller",
                           "/la_trajectory_controller",
                           "/rh_wr_trajectory_controller",
                           "/lh_wr_trajectory_controller"]

    republishers = RePubTrajectory(topics_to_republish, "/command_remapped", "/command")

    rospy.spin()
