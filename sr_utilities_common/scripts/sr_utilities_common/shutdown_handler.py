#!/usr/bin/python

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
import signal


class ShutdownHandler(object):
    def __init__(self, handled_object, handled_method, unattended=False):
        rospy.logwarn('Overriding default node shutdown handler!')
        signal.signal(signal.SIGINT, self.handler)
        self.handled_method = handled_method
        self.handled_object = handled_object
        self.command_on_shutdown = 'self.handled_object.{}'.format(self.handled_method)

    def handler(self, signum, frame):
        rospy.logwarn('Custom shutdown handler called.')
        exec(self.command_on_shutdown)
        rospy.signal_shutdown("SIGINT called")
