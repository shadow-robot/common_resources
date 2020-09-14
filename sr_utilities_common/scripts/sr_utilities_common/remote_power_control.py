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



class RemotePowerControl(object):
    def __init__(self, arm_power_ip):
        self._arm_power_ip = arm_power_ip
        rospy.Subscriber("power_arm_on", Bool, self.power_on_cb)
        rospy.Subscriber("power_arm_off", Bool, self.power_off_cb)

    def do_request(self, param, value, retries=10):
        request_string = self._arm_power_ip + "/setpara[" + param + "]=" + value
        i = 0
        r = requests.get(request_string)
        while (r.status_code != requests.codes.ok) || (i < retries):
            r = requests.get(request_string)   
            i = i + 1         

    def power_on(self):
        self.do_request(45, 1)
        rospy.sleep(0.2)
        self.do_request(45, 0)

    def power_off(self):
        self.do_request(46, 1)
        rospy.sleep(0.2)
        self.do_request(46, 0)

    def power_on_cb(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if data.data == True:
            rospy.loginfo("powering on...")
            self.power_on

    def power_off_cb(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if data.data == True:
            rospy.loginfo("powering off...")
            self.power_off


if __name__ == "__main__":
    ip="http://192.168.1.105"
    rospy.init_node('remote_power_control', anonymous=True)
    remote_power_control = RemotePowerControl(ip)
