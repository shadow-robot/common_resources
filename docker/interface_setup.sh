#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/user/projects/shadow_robot/base_deps/devel/setup.bash
source /home/user/projects/shadow_robot/base/devel/setup.bash
roscd sr_interface && git checkout F#SRC-318/arm_with_box
roscd sr_blockly_blocks && git checkout F#SRC-318/new_blocks
/usr/bin/terminator
