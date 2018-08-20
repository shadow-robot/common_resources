#!/usr/bin/python

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
