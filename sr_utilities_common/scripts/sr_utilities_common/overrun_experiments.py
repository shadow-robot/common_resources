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

# example use: ./overrun_experiments.py -ht hand_e -t 60

import rospy
import argparse
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from sr_robot_msgs.msg import EthercatDebug
import csv
import os


class OverrunExperiment(object):
    def __init__(self, hand_type, time):
        self.hand_type = hand_type
        self.time = time
        self.supported_hand_types = ['hand_e', 'hand_h']
        self.num_of_drops = 0
        self.iterations = 0
        self.overrun_average = 0
        self.drop_average = 0

    def get_recent_overruns_by_regex(self, msg):
        for status in msg.status:
            for value_dict in status.values:
                if 'Recent Control Loop Overruns' == value_dict.key:
                    return value_dict.value
        raise ValueError("\'Recent Control Loop overruns\' not present in the topic!")

    def overruns_callback_hand_h(self, data):
        overrun = self.get_recent_overruns_by_regex(data)
        rospy.loginfo(overrun)
        self.num_of_drops = sum(int(data.status[idx].values[8].value) for idx in range(4, 7))

        with open("overruns_data.txt", "a+") as myfile:
            myfile.write(overrun + "\t" + str(self.num_of_drops))
            myfile.write("\n")
        self.overrun_average += int(overrun)
        self.drop_average += self.num_of_drops
        self.num_of_drops = 0
        self.iterations += 1

    def overruns_callback_hand_e(self, data):
        overrun = self.get_recent_overruns_by_regex(data)
        with open("overruns_data.txt", "a+") as myfile:
            myfile.write(overrun + "\t" + str(self.num_of_drops))
            myfile.write("\n")
        self.overrun_average += int(float(overrun))
        self.drop_average += self.num_of_drops
        self.num_of_drops = 0
        self.iterations += 1

    def drops_callback_hand_e(self, data):
        if (0 == data.sensors[10]):
            self.num_of_drops += 1

    def listener(self):
        if not self.hand_type:
            raise ValueError('Please specify hand type using -ht (--hand_type) flag! (e.g. -ht hand_e or -ht hand_h)')
        elif self.hand_type not in self.supported_hand_types:
            raise ValueError('Unrecognized hand type: {}!'.format(self.hand_type))
        elif self.time < 1:
            raise ValueError('Please specify the experiment duration in seconds with -t 60 for example')

        rospy.loginfo("Your data is being recorded, please wait for " + str(self.time) + " seconds")
        if 'hand_h' == self.hand_type:
            rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.overruns_callback_hand_h)
        elif 'hand_e' == self.hand_type:
            rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.overruns_callback_hand_e)
            rospy.Subscriber("/rh/debug_etherCAT_data", EthercatDebug, self.drops_callback_hand_e)
        while (self.iterations < self.time) and (not rospy.is_shutdown()):
            rospy.sleep(0.1)
        rospy.loginfo("Your data has been recorded to ./overruns_data.txt file.")
        self.overrun_average = self.overrun_average / (1.0*self.time)
        self.drop_average = self.drop_average / (1.0*self.time)
        rospy.loginfo("Overrun average: "+str(self.overrun_average)+" Drop average: "+str(self.drop_average))


if __name__ == '__main__':
    if os.path.exists("overruns_data.txt"):
        os.remove("overruns_data.txt")
    rospy.init_node('overruns_experiment', anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument('-ht', '--hand_type', type=str, help='Hand type, e.g. hand_e or hand_h')
    parser.add_argument('-t', '--time', type=int, help='Number of seconds for the experiment to run')
    args = parser.parse_args()
    oe = OverrunExperiment(args.hand_type, args.time)
    oe.listener()
