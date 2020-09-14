#!/usr/bin/env python

# Copyright 2020 Shadow Robot Company Ltd.
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
import os
import rosservice
import sys
import requests
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry
from std_msgs.msg import Bool


class RemotePowerControl(object):
    def __init__(self, arm_power_ip, side="right"):
        self._on_off_delay = 0.4
        self._arm_power_ip = arm_power_ip
        rospy.Subscriber(side + "_power_arm_on", Bool, self.power_on_cb)
        rospy.Subscriber(side + "_power_arm_off", Bool, self.power_off_cb)
        while not rospy.is_shutdown():
            rospy.spin()

    def requests_retry_session(self, retries=3, backoff_factor=0.3, status_forcelist=(500, 502, 504), session=None):
        session = session or requests.Session()
        retry = Retry(
            total=retries,
            read=retries,
            connect=retries,
            backoff_factor=backoff_factor,
            status_forcelist=status_forcelist,
        )
        adapter = HTTPAdapter(max_retries=retry)
        session.mount('http://', adapter)
        session.mount('https://', adapter)
        return session

    def power_on(self):
        response = self.requests_retry_session().get(self._arm_power_ip + '/setpara[45]=1')
        rospy.sleep(self._on_off_delay)
        response = self.requests_retry_session().get(self._arm_power_ip + '/setpara[45]=0')
        rospy.sleep(self._on_off_delay)

    def power_off(self):
        response = self.requests_retry_session().get(self._arm_power_ip + '/setpara[46]=1')
        rospy.sleep(self._on_off_delay)
        response = self.requests_retry_session().get(self._arm_power_ip + '/setpara[46]=0')
        rospy.sleep(self._on_off_delay)

    def power_on_cb(self, data):
        if data.data == True:
            rospy.loginfo("powering on...")
            self.power_on()

    def power_off_cb(self, data):
        if data.data == True:
            rospy.loginfo("powering off...")
            self.power_off()


if __name__ == "__main__":
    ip="http://192.168.1.105"
    side="right"
    rospy.init_node('remote_power_control', anonymous=True)
    remote_power_control = RemotePowerControl(ip, side)


