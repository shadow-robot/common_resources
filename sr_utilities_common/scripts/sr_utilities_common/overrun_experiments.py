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

# example use: ./overrun_experiments.py -ht hand_e -t 60 -id lh

import os
import argparse
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sr_robot_msgs.msg import EthercatDebug


def get_recent_overruns_by_regex(msg):
    for status in msg.status:
        for value_dict in status.values:
            if value_dict.key == 'Recent Control Loop Overruns':
                return value_dict.value
    raise ValueError("\'Recent Control Loop overruns\' not present in the topic!")


class OverrunExperiment:
    def __init__(self, hand_type, time, hand_id):
        self.hand_type = hand_type
        self.hand_id = hand_id
        self.time = time
        self.supported_hand_types = ['hand_e', 'hand_h']
        self.num_of_drops = 0
        self.iterations = 0
        self.overrun_average = 0
        self.drop_average = 0

    def overruns_callback_hand_h(self, data):
        overrun = get_recent_overruns_by_regex(data)
        rospy.loginfo(overrun)
        self.num_of_drops = sum(int(data.status[idx].values[8].value) for idx in range(4, 7))

        with open("overruns_data.txt", "a+", encoding="utf-8") as myfile:
            myfile.write(overrun + "\t" + str(self.num_of_drops))
            myfile.write("\n")
        self.overrun_average += int(overrun)
        self.drop_average += self.num_of_drops
        self.num_of_drops = 0
        self.iterations += 1

    def overruns_callback_hand_e(self, data):
        overrun = get_recent_overruns_by_regex(data)
        with open("overruns_data.txt", "a+", encoding='UTF-8') as myfile:
            myfile.write(overrun + "\t" + str(self.num_of_drops))
            myfile.write("\n")
        self.overrun_average += int(float(overrun))
        self.drop_average += self.num_of_drops
        self.num_of_drops = 0
        self.iterations += 1

    def drops_callback_hand_e(self, data):
        if data.sensors[10] == 0:
            self.num_of_drops += 1

    def listener(self):
        if not self.hand_type:
            raise ValueError('Please specify hand type using -ht (--hand_type) flag! (e.g. -ht hand_e or -ht hand_h)')
        if self.hand_type not in self.supported_hand_types:
            raise ValueError(f'Unrecognized hand type: {self.hand_type}!')
        if self.time < 1:
            raise ValueError('Please specify the experiment duration in seconds with -t 60 for example')
        try:
            rospy.wait_for_message(f"/{self.hand_id}/debug_etherCAT_data", EthercatDebug, timeout=2)
        except rospy.exceptions.ROSException:
            rospy.logerr(f"Cannot find hand id: {self.hand_id}")
            raise
        rospy.loginfo("Your data is being recorded, please wait for " + str(self.time) + " seconds")
        if self.hand_type == 'hand_h':
            rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.overruns_callback_hand_h)
        elif self.hand_type == 'hand_e':
            rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.overruns_callback_hand_e)
            rospy.Subscriber(f"/{self.hand_id}/debug_etherCAT_data", EthercatDebug, self.drops_callback_hand_e)
        while (self.iterations < self.time) and (not rospy.is_shutdown()):
            rospy.sleep(0.1)
        rospy.loginfo("Your data has been recorded to ./overruns_data.txt file.")
        self.overrun_average = self.overrun_average / (1.0*self.time)
        self.drop_average = self.drop_average / (1.0*self.time)
        rospy.loginfo(f"Overrun average: {self.overrun_average}. Drop average: {self.drop_average}")


if __name__ == '__main__':
    if os.path.exists("overruns_data.txt"):
        os.remove("overruns_data.txt")
    rospy.init_node('overruns_experiment', anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument('-ht', '--hand_type', type=str, help='Hand type, e.g. hand_e or hand_h')
    parser.add_argument('-t', '--time', type=int, help='Number of seconds for the experiment to run')
    parser.add_argument('-id', '--hand_id', type=str, help='Hand id, e.g. rh or lh', default="rh")
    args = parser.parse_args()
    oe = OverrunExperiment(args.hand_type, args.time, args.hand_id)
    oe.listener()
