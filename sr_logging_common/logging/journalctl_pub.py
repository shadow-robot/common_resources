#!/usr/bin/env python

import rospy
from sr_msgs_common.msg import journal_log
import subprocess
import select
import os


if __name__ == "__main__":
    rospy.init_node('journalctl_pub', anonymous=True)
    pub = rospy.Publisher('journalctl_log', journal_log, queue_size=10)
    
    if (os.path.isdir("/host_run")):
        args = ['journalctl', '--directory', '/host_run/log/journal/', '--lines', '0', '--follow']
    else:
        args = ['journalctl', '--lines', '0', '--follow']
    f = subprocess.Popen(args, stdout=subprocess.PIPE)
    p = select.poll()
    p.register(f.stdout)

    while not rospy.is_shutdown():
        if p.poll(100):
            journal_log_line = f.stdout.readline()
            if journal_log_line:
                journal_msg = journal_log()
                journal_msg.data = journal_log_line.strip()
                journal_msg.timestamp = rospy.get_rostime()
                pub.publish(journal_msg)
