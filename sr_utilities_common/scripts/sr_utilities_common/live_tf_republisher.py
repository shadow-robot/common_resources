#!/usr/bin/env python3

# Copyright 2024 Shadow Robot Company Ltd.
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

import os
import yaml
import rospy
import tf2_ros
import tf2_msgs


class SrLiveTfRepublisher:
    def __init__(self, config_file):
        loaded_config = self._load_and_validate_config(config_file)
        self._parent_child_frames = []
        for parent_child_pair in loaded_config['parent_to_child_frames']:
            for parent, child in parent_child_pair.items():
                self._parent_child_frames.append((parent, child))

        output_topic = loaded_config['output_topic']
        frequency = loaded_config['frequency']
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._broadcaster = tf2_ros.TransformBroadcaster()
        self._broadcaster.pub_tf.name = output_topic
        self._broadcaster.pub_tf.__init__(self._broadcaster.pub_tf.name,  # pylint: disable=W0233
                                          tf2_msgs.msg.TFMessage,
                                          queue_size=100)
        self._timer = rospy.Timer(rospy.Duration(1/frequency), self._timer_callback)

    @staticmethod
    def _load_and_validate_config(config_file):
        if not os.path.exists(config_file):
            raise FileNotFoundError(f'File {config_file} not found')
        with open(config_file, 'r', encoding='utf-8') as opened_file:
            loaded_config = yaml.safe_load(opened_file)
        if 'sr_tf_live_republisher' not in loaded_config:
            raise ValueError("Config file must have 'sr_tf_live_republisher' section at the top level")
        if 'parent_to_child_frames' not in loaded_config['sr_tf_live_republisher']:
            raise ValueError('Config file must contain parent_to_child_frames under sr_tf_live_republisher')
        if len(loaded_config['sr_tf_live_republisher']['parent_to_child_frames']) == 0:
            raise ValueError('parent_to_child_frames section must contain at least one parent to child frame mapping')
        if 'output_topic' not in loaded_config['sr_tf_live_republisher']:
            raise ValueError('Config file must contain output_topic section under sr_tf_live_republisher')
        if 'frequency' not in loaded_config['sr_tf_live_republisher']:
            raise ValueError('Config file must contain frequency section under sr_tf_live_republisher')
        return loaded_config['sr_tf_live_republisher']

    def _timer_callback(self, _):
        try:
            transforms = [self._tf_buffer.lookup_transform(parent_child_pair[0], parent_child_pair[1], rospy.Time())
                          for parent_child_pair in self._parent_child_frames]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        else:
            for trans in transforms:
                self._broadcaster.sendTransform(trans)


if __name__ == '__main__':
    rospy.init_node('tf_live_filter', anonymous=True)
    input_config_file = rospy.get_param('~config_file')
    republisher = SrLiveTfRepublisher(input_config_file)
    rospy.spin()
