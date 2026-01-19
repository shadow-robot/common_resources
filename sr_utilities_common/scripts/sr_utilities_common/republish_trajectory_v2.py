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

'''
WHEN TO USE:
Use this script when you have a rosbag and want to play the trajectory data on a hand or hand and arm system.
This tool is intended to work on all hand and arm variations (unimanual, bimanual, hand only, etc).

HOW TO USE:
1. Start the hardware control loop of the system
2. Start the republisher node
3. Play the rosbag you want to replay the trajectory from. Ensure you define all topics you want to play and remap
   them. An example command would be:
    ```
        rosbag play my_trajectory.bag --topics /rh_trajectory_controller/command /ra_trajectory_controller/command
        /rh_trajectory_controller/command:=/rh_trajectory_controller/command_remapped
        /ra_trajectory_controller/command:=/ra_trajectory_controller/command_remapped
    ```
'''

from typing import List
from threading import Lock
from functools import partial
import rospy
from trajectory_msgs.msg import JointTrajectory


class RePubTrajectory:

    # Amount by which republished msgs are shifted to the future to ensure they arrive at the
    # controller before their timestamp
    FUTURE_SHIFT = 10 / 1000

    # Time in seconds after which the node will reset if not topics are received. Node reset means the
    # "topics_first_msg_timestamp" variable will be reset to None
    NODE_RESET_TIME = 3

    def __init__(self, topics: List[str], subscribed_subtopic: str, published_subtopic: str):
        self._mutex = Lock()
        self._topics_first_msg_timestamp = None  # Timestamp of first msg received amongst all topics to republish
        self._current_time_of_first_msg = None  # Current time when first msg was received
        self._watchdog_current_time = rospy.get_time()

        self._traj_pubs = []
        self._bag_tf_subs = []

        for topic in topics:
            traj_pub = rospy.Publisher(topic + published_subtopic, JointTrajectory, queue_size=10)
            bag_tf_sub = rospy.Subscriber(topic + subscribed_subtopic, JointTrajectory,
                                          partial(self._bag_traj_cb, traj_pub))
            # Using a partial here to reuse the same callback method with multiple publishers

            self._traj_pubs.append(traj_pub)
            self._bag_tf_subs.append(bag_tf_sub)

    def _bag_traj_cb(self, republisher: rospy.Publisher, data: JointTrajectory):
        with self._mutex:
            self._watchdog_current_time = rospy.get_time()

            if self._topics_first_msg_timestamp is None:
                self._topics_first_msg_timestamp = data.header.stamp
                self._current_time_of_first_msg = rospy.Time.now()

        data.header.stamp -= self._topics_first_msg_timestamp
        data.header.stamp += self._current_time_of_first_msg
        data.header.stamp += rospy.Duration.from_sec(self.FUTURE_SHIFT)  # Shift timestamp to the future

        republisher.publish(data)

    def spin(self):
        while not rospy.is_shutdown():
            if self._topics_first_msg_timestamp is None:
                continue

            if rospy.get_time() - self._watchdog_current_time > self.NODE_RESET_TIME:
                rospy.logwarn(f"Resetting node. No bag topics received in the last {self.NODE_RESET_TIME} seconds")

                with self._mutex:
                    self._topics_first_msg_timestamp = None

            rospy.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node("republish_trajectory")

    topics_to_republish = ["/rh_trajectory_controller",
                           "/lh_trajectory_controller",
                           "/ra_trajectory_controller",
                           "/la_trajectory_controller",
                           "/rh_wr_trajectory_controller",
                           "/lh_wr_trajectory_controller"]

    republishers = RePubTrajectory(topics_to_republish, "/command_remapped", "/command")

    republishers.spin()
