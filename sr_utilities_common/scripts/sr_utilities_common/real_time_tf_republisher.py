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

import rospy
import tf
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


class RealTimeTfRepublisher(object):
    def __init__(self, bag_tf_topic_name, tf_name_regex="", tcp_nodelay=True):
        self._tf_name_regex = tf_name_regex
        self._bag_tf_sub = rospy.Subscriber(bag_tf_topic_name, TFMessage,
                                            self._bag_tf_cb, tcp_nodelay=tcp_nodelay)
        self._tf_republisher = rospy.Publisher("/tf", TFMessage, queue_size=10)

    def _bag_tf_cb(self, data):
        list_of_tfs_to_publish = TFMessage()
        publish = False
        for tf in data.transforms:
            if self._tf_name_regex in tf.child_frame_id:
                tf_to_be_republished = TransformStamped()
                tf_to_be_republished = tf
                tf_to_be_republished.header.stamp = rospy.Time.now()
                list_of_tfs_to_publish.transforms.append(tf_to_be_republished)
                publish = True
        if publish:
            self._tf_republisher.publish(list_of_tfs_to_publish)

if __name__ == "__main__":
    rospy.init_node("real_time_tf_republisher")
    bag_tf_topic_name = rospy.get_param('~bag_tf_topic_name')
    real_time_tf_republisher = RealTimeTfRepublisher(bag_tf_topic_name)
    rospy.spin()
