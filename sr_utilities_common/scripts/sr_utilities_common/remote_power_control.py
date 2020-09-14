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

    def power_on(self):
        r = requests.get(self._arm_power_ip + "/setpara[45]=1")
        rospy.logwarn("%s", r)
        rospy.sleep(0.5)
        r = requests.get(self._arm_power_ip + "/setpara[45]=0")
        rospy.logwarn("%s", r)
        rospy.sleep(1.0)        

    def power_off(self):
        r = requests.get(self._arm_power_ip + "/setpara[46]=1")
        rospy.logwarn("%s", r)
        rospy.sleep(0.5)
        r = requests.get(self._arm_power_ip + "/setpara[46]=0")
        rospy.logwarn("%s", r)
        rospy.sleep(1.0)        

    def power_on_cb(self, message):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if data.data == True:
            rospy.loginfo("powering on...")
            self.power_on

    def power_off_cb(self, message):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if data.data == True:
            rospy.loginfo("powering off...")
            self.power_off


if __name__ == "__main__":
    ip="192.168.1.105"
    rospy.init_node('remote_power_control', anonymous=True)
    remote_power_control = RemotePowerControl(ip)
