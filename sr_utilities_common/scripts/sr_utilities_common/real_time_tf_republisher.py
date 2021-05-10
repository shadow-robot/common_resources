#!/usr/bin/env python3

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

from __future__ import absolute_import
import re
import rospy
import threading
import tf
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


class RealTimeTfRepublisher(object):
    def __init__(self, bag_tf_topic_name, tf_name_regexes=[], tcp_nodelay=True):
        rospy.loginfo("TF republisher will republish TFs that match these regexes: {}".format(tf_name_regexes))
        self._tf_name_regexes = [re.compile(x) for x in tf_name_regexes]
        self._tf_republisher = rospy.Publisher("/tf", TFMessage, queue_size=10)
        self._published_tfs = []
        self._matched_regexes = []
        self._bag_tf_sub = rospy.Subscriber(bag_tf_topic_name, TFMessage,
                                            self._bag_tf_cb, tcp_nodelay=tcp_nodelay)
        threading.Timer(10, self._check_matched_regexes, [10]).start()

    def _bag_tf_cb(self, data):
        list_of_tfs_to_publish = TFMessage()
        for tf in data.transforms:
            if (tf.child_frame_id in [x.child_frame_id for x in list_of_tfs_to_publish.transforms]):
                # This TF is already going to be republished, either more than one regex matches it, or originally
                # bagged TF tree is invalid (TFs can't have multiple parents)
                continue
            for regex in self._tf_name_regexes:
                if regex.match(tf.child_frame_id) is not None:
                    if (tf.child_frame_id not in self._published_tfs):
                        rospy.loginfo("Republishing matching TF: {}".format(tf.child_frame_id))
                        self._published_tfs.append(tf.child_frame_id)
                    if (regex not in self._matched_regexes):
                        self._matched_regexes.append(regex)
                    tf_to_be_republished = tf
                    tf_to_be_republished.header.stamp = rospy.Time.now()
                    list_of_tfs_to_publish.transforms.append(tf_to_be_republished)
        if len(list_of_tfs_to_publish.transforms) > 0:
            self._tf_republisher.publish(list_of_tfs_to_publish)

    def _check_matched_regexes(self, timeout):
        unmatched_regexes = []
        for regex in self._tf_name_regexes:
            if regex not in self._matched_regexes:
                unmatched_regexes.append(regex)
        if unmatched_regexes:
            rospy.logwarn("The following TFs were not found in the first {}s of the supplied ROSBag: {}".format(
                timeout, [unmatched_regex.pattern for unmatched_regex in unmatched_regexes]))


if __name__ == "__main__":
    rospy.init_node("real_time_tf_republisher")
    bag_tf_topic_name = rospy.get_param('~bag_tf_topic_name', 'tf_bag')
    tf_name_regexes = rospy.get_param('~tf_name_regexes', [])
    tcp_nodelay = rospy.get_param('~tcp_nodelay', True)
    real_time_tf_republisher = RealTimeTfRepublisher(bag_tf_topic_name, tf_name_regexes, tcp_nodelay)
    rospy.spin()
