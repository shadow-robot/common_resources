#!/usr/bin/env python3

# Copyright 2019-2022 Shadow Robot Company Ltd.
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

import re
import threading
import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage


class RealTimeTfRepublisher:
    def __init__(self, bag_tf_topic_name, tf_name_regexes=None, tcp_nodelay=True):
        rospy.loginfo("TF republisher will republish TFs that match these regexes: {}".format(tf_name_regexes))
        if not tf_name_regexes:
            tf_name_regexes = []
        self._tf_name_regexes = [re.compile(tf_regex) for tf_regex in tf_name_regexes]
        self._tf_republisher = tf2_ros.TransformBroadcaster()
        self._published_tfs = []
        self._matched_regexes = []
        self._last_published_times = {}
        self._bag_tf_sub = rospy.Subscriber(bag_tf_topic_name, TFMessage,
                                            self._bag_tf_cb, tcp_nodelay=tcp_nodelay, queue_size=1)
        threading.Timer(10, self._check_matched_regexes, [10]).start()

    def _bag_tf_cb(self, data):
        list_of_tfs_to_publish = []
        current_time = rospy.Time.now()
        for transform in data.transforms:
            if (transform.child_frame_id in [tf_to_publish.child_frame_id for tf_to_publish in list_of_tfs_to_publish]):
                # This TF is already going to be republished, either more than one regex matches it, or originally
                # bagged TF tree is invalid (TFs can't have multiple parents)
                continue
            if transform.child_frame_id in self._last_published_times and \
               self._last_published_times[transform.child_frame_id] == current_time:
                # This TF has already been published at this timestamp; don't republish (to avoid TF_REPEATED DATA)
                continue
            for regex in self._tf_name_regexes:
                if regex.match(transform.child_frame_id) is not None:
                    if transform.child_frame_id not in self._published_tfs:
                        rospy.loginfo(f"Republishing matching TF: {transform.child_frame_id}")
                        self._published_tfs.append(transform.child_frame_id)
                    if regex not in self._matched_regexes:
                        self._matched_regexes.append(regex)
                    tf_to_be_republished = transform
                    self._last_published_times[transform.child_frame_id] = current_time
                    tf_to_be_republished.header.stamp = current_time
                    list_of_tfs_to_publish.append(tf_to_be_republished)
        if len(list_of_tfs_to_publish) > 0:
            self._tf_republisher.sendTransform(list_of_tfs_to_publish)

    def _check_matched_regexes(self, timeout):
        unmatched_regexes = []
        for regex in self._tf_name_regexes:
            if regex not in self._matched_regexes:
                unmatched_regexes.append(regex)
        if unmatched_regexes:
            rospy.logwarn(f"The following TFs were not found in the first {timeout}s \
                            of the supplied ROSBag: \
                            {[unmatched_regex.pattern for unmatched_regex in unmatched_regexes]}")


if __name__ == "__main__":
    rospy.init_node("real_time_tf_republisher")
    bag_tf_topic_name_param = rospy.get_param('~bag_tf_topic_name', 'tf_bag')
    tf_name_regexes_param = rospy.get_param('~tf_name_regexes', [])
    tcp_nodelay_param = rospy.get_param('~tcp_nodelay', True)
    real_time_tf_republisher = RealTimeTfRepublisher(bag_tf_topic_name_param, tf_name_regexes_param, tcp_nodelay_param)
    rospy.spin()
