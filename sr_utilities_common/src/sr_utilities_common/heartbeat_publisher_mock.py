#!/usr/bin/env python
#
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import rospkg
import sys
import termios
import tty
from std_msgs.msg import Bool
from threading import Thread, Lock


class HeartbeatPublisherMock(object):
    def __init__(self):
        self._threads_running = None
        self._input_thread = None
        self._heartbeat_status = False

        self._threading_lock = Lock()
        self._teleop_heartbeat_publisher = rospy.Publisher('/teleop_heartbeat', Bool, queue_size=10)

        self._start_heartbeat_publishing_thread()

    def _heartbeat_publishing_thread(self):
        rate = rospy.Rate(1000)
        status_count = 1
        while self._threads_running:
            if (status_count == 10):
                self._publish_heartbeat_status()
                status_count = 0
            status_count += 1
            rate.sleep()

    def _start_heartbeat_publishing_thread(self):
        self._threads_running = True
        self._heartbeat_thread = Thread(target=self._heartbeat_publishing_thread)
        self._heartbeat_thread.start()

    def _publish_heartbeat_status(self):
        self._teleop_heartbeat_publisher.publish(data=self._heartbeat_status)

    def change_heartbeat_status(self, bool_data=False):
        self._heartbeat_status = bool_data

    def _get_input(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def stop(self):
        rospy.loginfo("Stopping all threads...")
        self._threads_running = False
        self._heartbeat_thread.join()


if __name__ == "__main__":
    rospy.init_node('heartbeat_mock_node', anonymous=True)
    heartbeat = HeartbeatPublisherMock()

    CONST_ESC_KEY_HEX_VALUE = '0x1b'
    while heartbeat._threads_running:
        rospy.loginfo("Press any key to toggle teleop heartbeat. Press ESC to exit.")
        input_val = heartbeat._get_input()
        if CONST_ESC_KEY_HEX_VALUE == hex(ord(input_val)):
            heartbeat.stop()
            sys.exit(0)
        else:
            if heartbeat._heartbeat_status == False:
                heartbeat._heartbeat_status = True
            else:
                heartbeat._heartbeat_status = False
            heartbeat.change_heartbeat_status(heartbeat._heartbeat_status)
