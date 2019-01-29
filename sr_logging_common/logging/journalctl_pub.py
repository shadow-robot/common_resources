#!/usr/bin/env python

import rospy
from sr_msgs_common.msg import journal_log_msg
import subprocess
import select


if __name__ == "__main__":
    rospy.init_node('journalctl_pub', anonymous=True)
    pub = rospy.Publisher('journalctl_pub', journal_log_msg, queue_size=10)

    args = ['journalctl', '--lines', '0', '--follow']
    f = subprocess.Popen(args, stdout=subprocess.PIPE)
    p = select.poll()
    p.register(f.stdout)

    while not rospy.is_shutdown():
        if p.poll(100):
            journal_log_line = f.stdout.readline()
            if journal_log_line:
                journal_msg = journal_log_msg()
                journal_msg.data = journal_log_line.strip()
                journal_msg.timestamp = rospy.get_rostime()
                pub.publish(journal_msg)
