#!/usr/bin/env python
import os
import rospy
import time


def getFolderSize(folder):
    if os.path.isdir(folder):
        total_size = os.path.getsize(folder)
        for item in os.listdir(folder):
            itempath = os.path.join(folder, item)
            if os.path.isfile(itempath):
                total_size += os.path.getsize(itempath)
            elif os.path.isdir(itempath):
                total_size += getFolderSize(itempath)
    else:
        total_size = 0
    return total_size

if __name__ == '__main__':
    rospy.init_node('core_dump_limit', anonymous=True)
    desired_size = rospy.get_param('~desired_folder_size', 1000000000)
    path = rospy.get_param('~core_dump_path', '/home/user/.ros/log/core_dumps')
    while not rospy.is_shutdown():
        if getFolderSize(path) > desired_size:
            oldest = min(os.listdir(path), key=lambda f: os.path.getctime("{}/{}".format(path, f)))
            rospy.loginfo("Core dump size greater than limit. Removing oldest file: " + oldest)
            os.remove(path + '/' + oldest)
        time.sleep(5)
