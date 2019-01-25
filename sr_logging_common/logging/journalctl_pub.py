#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys
import subprocess
import select


if __name__ == "__main__":
    rospy.init_node('journalctl_pub', anonymous=True)
    pub = rospy.Publisher('journalctl_pub', String, queue_size=10)

    args = ['journalctl', '--lines', '0', '--follow', '_SYSTEMD_UNIT=docker.service']
    f = subprocess.Popen(args, stdout=subprocess.PIPE)
    p = select.poll()
    p.register(f.stdout)

    while not rospy.is_shutdown():
        if p.poll(100):
            line = f.stdout.readline()
            rospy.loginfo(line.strip())
            pub.publish(line.strip())
