#!/usr/bin/python3

# Copyright 2018 Open Source Robotics Foundation
# Copyright 2019, 2022 Shadow Robot Company Ltd.
#
# Creative Commons Attribution 3.0
# https://creativecommons.org/licenses/by/3.0/
# Copied from http://wiki.ros.org/rosbag/Cookbook
# pylint: disable=E1101

from builtins import round
import sys
import time
import subprocess
import os
import argparse
import math
from shutil import move
import yaml
import rosbag


def status(length, percent):
    sys.stdout.write('\x1B[2K')  # Erase entire current line
    sys.stdout.write('\x1B[0E')  # Move to the beginning of the current line
    progress = "Progress: ["
    for i in range(0, length):
        if i < length * percent:
            progress += '='
        else:
            progress += ' '
    progress += "] " + str(round(percent * 100.0, 2)) + "%"
    sys.stdout.write(progress)
    sys.stdout.flush()


def main(args):
    parser = argparse.ArgumentParser(
        description='Reorder a bagfile based on header timestamps.')
    parser.add_argument('bagfile', nargs=1, help='input bag file')
    parser.add_argument('--max-offset', nargs=1,
                        help='max time offset (sec) to correct.', default='60', type=float)
    args = parser.parse_args()

    # Get bag duration

    bagfile = args.bagfile[0]

    info_dict = yaml.safe_load(subprocess.Popen(  # pylint: disable=R1732
        ['rosbag', 'info', '--yaml', bagfile], stdout=subprocess.PIPE).communicate()[0])
    duration = info_dict['duration']
    start_time = info_dict['start']

    orig = os.path.splitext(bagfile)[0] + ".orig.bag"

    move(bagfile, orig)

    with rosbag.Bag(bagfile, 'w') as outbag:

        last_time = time.clock()
        for topic, msg, time_point in rosbag.Bag(orig).read_messages():

            if time.clock() - last_time > .1:
                percent = (time_point.to_sec() - start_time) / duration
                status(40, percent)
                last_time = time.clock()

            # This also replaces tf timestamps under the assumption
            # that all transforms in the message share the same timestamp
            if topic == "/tf" and msg.transforms:
                diff = math.fabs(
                    msg.transforms[0].header.stamp.to_sec() - time_point.to_sec())
                outbag.write(
                    topic, msg, msg.transforms[0].header.stamp if diff < args.max_offset else time_point)
            elif msg._has_header:  # pylint: disable=W0212
                diff = math.fabs(msg.header.stamp.to_sec() - time_point.to_sec())
                outbag.write(topic, msg, msg.header.stamp if diff <
                             args.max_offset else time_point)
            else:
                outbag.write(topic, msg, time_point)
    status(40, 1)
    print("\ndone")


if __name__ == "__main__":
    main(sys.argv[1:])
