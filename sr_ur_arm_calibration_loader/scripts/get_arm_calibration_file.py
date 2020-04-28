#!/usr/bin/env python
#
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.


import rospy
import sys

input_file = sys.argv[1]
with open(input_file, 'r') as outFile:
    #outFile.read()
    print outFile.read()

