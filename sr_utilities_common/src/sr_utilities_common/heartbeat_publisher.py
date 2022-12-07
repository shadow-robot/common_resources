#!/usr/bin/env python3

# Copyright 2020, 2022 Shadow Robot Company Ltd.
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

from builtins import input
import argparse
from threading import Thread, Lock
import rospy
from std_msgs.msg import Bool


class HeartbeatPublisher:
    def __init__(self, topic_name='heartbeat', publishing_rate=20):
        self._publishing_thread_running = False
        self._heartbeat_status = False
        self.rate = rospy.Rate(publishing_rate)

        self._threading_lock = Lock()
        self._teleop_heartbeat_publisher = rospy.Publisher(topic_name, Bool, queue_size=10)

        self._start_heartbeat_publishing_thread()

    def _heartbeat_publishing_thread(self):
        while self._publishing_thread_running:
            self._teleop_heartbeat_publisher.publish(data=self._heartbeat_status)
            self.rate.sleep()

    def _start_heartbeat_publishing_thread(self):
        rospy.loginfo("Starting thread...")
        self._publishing_thread_running = True
        self._heartbeat_thread = Thread(target=self._heartbeat_publishing_thread)
        self._heartbeat_thread.start()

    def change_heartbeat_status(self, bool_data):
        self._heartbeat_status = bool_data

    def stop(self):
        rospy.loginfo("Stopping thread...")
        self._publishing_thread_running = False
        self._heartbeat_thread.join()


if __name__ == "__main__":
    rospy.init_node('heartbeat_publisher_node', anonymous=True)

    parser = argparse.ArgumentParser(description='A script to publish and toggle a heartbeat topic.',
                                     add_help=True,
                                     usage='%(prog)s [-h] --topic_name TOPIC_NAME, --publishing_rate PUBLISHING_RATE',
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--topic_name', dest='topic_name',
                        default='heartbeat',
                        help="TOPIC_NAME is the topic you want to publish Bool data to.")
    parser.add_argument('--publishing_rate', dest='publishing_rate',
                        type=int, default=20,
                        help="PUBLISHING_RATE is the rate you want to publish the topic.")
    args = parser.parse_args()
    heartbeat = HeartbeatPublisher(args.topic_name, args.publishing_rate)

    status = False
    while not rospy.is_shutdown():
        input_val = input("Press [RETURN] to toggle /" + args.topic_name +
                          " at " + str(args.publishing_rate) + "Hz. " +
                          "Type 'exit' and execute to terminate. \n")
        if input_val == 'exit':
            break
        status = not status
        heartbeat.change_heartbeat_status(status)
    heartbeat.stop()
