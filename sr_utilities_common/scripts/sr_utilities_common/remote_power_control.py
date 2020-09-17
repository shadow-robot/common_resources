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
import requests
import actionlib
import os
import subprocess
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry
from std_msgs.msg import Bool

from sr_utilities_common.msg import PowerManagerAction



class RemotePowerControl(object):
    def __init__(self, name, arm_power_ip, arm_ip_address, side="right"):
        self._action_name = name
        self._on_off_delay = 0.4
        self._arm_power_ip = arm_power_ip
        self._http_arm_power_ip = 'http://' + arm_power_ip
        self._arm_data_ip = arm_ip_address
        rospy.Subscriber(side + "_arm_power_on", Bool, self.power_on_cb)
        rospy.Subscriber(side + "_arm_power_off", Bool, self.power_off_cb)

        self._as = actionlib.SimpleActionServer(self._action_name, PowerManagerAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        if self.does_ip_relay_respond():
            rospy.loginfo("Contacted " + side + " arm ip relay")
        else:
            rospy.logerr("Cannot reach arm ip relay at " + self._arm_power_ip)
        #while not rospy.is_shutdown():
            #rospy.spin()


    def execute_cb(self):
        pass

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
        response = self.requests_retry_session().get(self._http_arm_power_ip + '/setpara[45]=1')
        rospy.sleep(self._on_off_delay)
        response = self.requests_retry_session().get(self._http_arm_power_ip + '/setpara[45]=0')
        rospy.sleep(self._on_off_delay)

    def power_off(self):
        response = self.requests_retry_session().get(self._http_arm_power_ip + '/setpara[46]=1')
        rospy.sleep(self._on_off_delay)
        response = self.requests_retry_session().get(self._http_arm_power_ip + '/setpara[46]=0')
        rospy.sleep(self._on_off_delay)

    def power_on_cb(self, data):
        if data.data == True:
            if self.is_arm_off():
                rospy.loginfo("powering on...")
                self.power_on()
            else: 
                rospy.logwarn("arm already on, not powering up")

    def power_off_cb(self, data):
        if data.data == True:
            if self.is_arm_on():
                rospy.loginfo("powering off...")
                self.power_off()
            else:
                rospy.logwarn("Arm already off, not sending shutdown signal")

    def is_ping_successful(self, ip):
        command = 'fping -c1 -t500 ' + ip + ' 2>&1 >/dev/null'
        p = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
        stdout = p.communicate()[0]
        if (p.returncode == 0):
            return True
        else:
            return False

    def does_ip_relay_respond(self):
        return self.is_ping_successful(self._arm_power_ip)

    def is_arm_off(self):
        return not self.is_ping_successful(self._arm_data_ip)

    def is_arm_on(self):
        return self.is_ping_successful(self._arm_data_ip)

    def is_arm_booting(self):        
        while True:
            t = os.system('fping -c1 -t500 192.168.1.1')
            print t


if __name__ == "__main__":
    rospy.init_node('remote_power_control', anonymous=True)
    if rospy.has_param('~arm_ip'):
        arm_ip = rospy.get_param("~arm_ip")
    else:
        arm_ip = "192.168.1.1"

    if rospy.has_param('~arm_power_ip'):
        arm_power_ip = rospy.get_param("~arm_power_ip")
    else:
        arm_power_ip = "10.6.10.105"

    if rospy.has_param('~side'):
        side = rospy.get_param("~side")
    else:
        side = "right"
    ##ip_power_address = "http://" + arm_power_ip
    arm_address = arm_ip
    remote_power_control = RemotePowerControl(rospy.get_name(), arm_power_ip, arm_address, side)

