#!/usr/bin/env python3

# Copyright 2020-2021 Shadow Robot Company Ltd.
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

from __future__ import absolute_import

from controller_manager_msgs.srv import (ListControllers, LoadController,
                                         SwitchController, SwitchControllerRequest)
import rospy
from sr_utilities.hand_finder import HandFinder
from sr_robot_msgs.msg import ControlType
from sr_robot_msgs.srv import (ChangeControlType, RobotTeachMode,
                               RobotTeachModeRequest, RobotTeachModeResponse, SetTeachMode)

import rospy

def _change_ctrl(controllers, managed_controllers):
    success = True
    list_controllers = rospy.ServiceProxy(
        'controller_manager/list_controllers', ListControllers)
    try:
        resp1 = list_controllers()
    except rospy.ServiceException:
        success = False

    if success:
        controllers_to_stop = [
            c.name for c in resp1.controller if c.state == "running" and c.name in managed_controllers]
        all_loaded_controllers = [c.name for c in resp1.controller]

        controllers_to_start = controllers
        for running_controller in controllers_to_stop[:]:
            if running_controller in controllers_to_start:
                    controllers_to_stop.remove(running_controller)
                    controllers_to_start.remove(running_controller)

        rospy.logerr(str(controllers_to_stop))
        rospy.logerr(str(controllers_to_start))

        for load_control in controllers_to_start:
            if load_control not in all_loaded_controllers:
                try:
                    load_controllers = rospy.ServiceProxy(
                        'controller_manager/load_controller', LoadController)
                    resp1 = load_controllers(load_control)
                except rospy.ServiceException:
                    success = False
                if not resp1.ok:
                    success = False

        input("enter to stop current controllers...")

        rospy.logerr("to stop:\n" + str(controllers_to_stop))
        rospy.logerr("to start:\n" + str(controllers_to_start))

        switch_controllers = rospy.ServiceProxy(
            'controller_manager/switch_controller', SwitchController)
        resp1 = switch_controllers(
            [], controllers_to_stop, SwitchControllerRequest.STRICT, True, 0.0)
        
        if not resp1.ok:
            success = False

        input("enter to start controllers...")

        ### WORKS ####
        # for controller in controllers_to_start:
        #     print("starting: " + str(controller))
        #     resp2 = switch_controllers(
        #             [controller], [], SwitchControllerRequest.STRICT, True, 0.0)
            
        #     if not resp2.ok:
        #         success = False

        ### DOES NOT WORK ###
        resp2 = switch_controllers(
            controllers_to_start, [], SwitchControllerRequest.STRICT, True, 0.0)
        print("going")
        if not resp2.ok:
            success = False


    if not success:
        rospy.logwarn(
            "Failed to change some of the controllers. This is normal if this is not a 5 finger hand.")

    return success

if __name__ == "__main__":
    rospy.init_node("testing_node")

    # This section will only be used for hands

    joints = ["ffj0", "ffj3", "ffj4",
                        "mfj0", "mfj3", "mfj4",
                        "rfj0", "rfj3", "rfj4",
                        "lfj0", "lfj3", "lfj4", "lfj5",
                        "thj1", "thj2", "thj3", "thj4", "thj5",
                        "wrj1", "wrj2"]
    hand_joint_prefix = ["rh_"]
    robot_joint_prefixes = ["rh_"]
    hand_controllers = {
        "effort": ["sh_{0}{1}_effort_controller".format(hand_joint_prefix, joint)
                for joint in joints
                for hand_joint_prefix in robot_joint_prefixes],
        "position": ["sh_{0}{1}_position_controller".format(hand_joint_prefix, joint)
                    for joint in joints
                    for hand_joint_prefix in robot_joint_prefixes],
        "trajectory": ["rh_trajectory_controller", "rh_wr_trajectory_controller"],
        "mixed": ["sh_{0}{1}_mixed_position_velocity_controller".format(hand_joint_prefix, joint)
                for joint in joints
                for hand_joint_prefix in robot_joint_prefixes],
        "velocity": ["sh_{0}{1}_velocity_controller".format(hand_joint_prefix, joint)
                    for joint in joints
                    for hand_joint_prefix in robot_joint_prefixes],
        "stop": []}
    managed_hand_controllers = [
        cont for type_conts in hand_controllers.values() for cont in type_conts]

    print(str(_change_ctrl(hand_controllers["effort"], managed_hand_controllers)))    
