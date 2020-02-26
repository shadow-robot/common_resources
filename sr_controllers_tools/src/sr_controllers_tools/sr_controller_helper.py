#!/usr/bin/env python

# Copyright 2020 Shadow Robot Company Ltd.
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


from controller_manager_msgs.srv import (ListControllers, LoadController,
                                         SwitchController, SwitchControllerRequest)
import rospy
from sr_robot_msgs.msg import ControlType
from sr_robot_msgs.srv import (ChangeControlType, RobotTeachMode,
                               RobotTeachModeRequest, RobotTeachModeResponse, SetTeachMode)


class ControllerHelper(object):

    def __init__(self, robot_ids, robot_joint_prefixes, robot_joint_names):
        self.time_to_reload_params = 6.0
        self.robot_ids = robot_ids
        self.robot_joint_prefixes = robot_joint_prefixes

        # This section will only be used for hands
        self.joints = robot_joint_names
        self.hand_controllers = {
            "effort": ["sh_{0}{1}_effort_controller".format(hand_joint_prefix, joint)
                       for joint in self.joints
                       for hand_joint_prefix in self.robot_joint_prefixes],
            "position": ["sh_{0}{1}_position_controller".format(hand_joint_prefix, joint)
                         for joint in self.joints
                         for hand_joint_prefix in self.robot_joint_prefixes],
            "mixed": ["sh_{0}{1}_mixed_position_velocity_controller".format(hand_joint_prefix, joint)
                      for joint in self.joints
                      for hand_joint_prefix in self.robot_joint_prefixes],
            "velocity": ["sh_{0}{1}_velocity_controller".format(hand_joint_prefix, joint)
                         for joint in self.joints
                         for hand_joint_prefix in self.robot_joint_prefixes],
            "stop": []}
        self.managed_hand_controllers = [
            cont for type_conts in self.hand_controllers.itervalues() for cont in type_conts]

        # This section is for any robot
        self.trajectory_controllers = {
            "run": ["{0}_trajectory_controller".format(robot_id) for robot_id in self.robot_ids],
            "stop": []}
        self.managed_trajectory_controllers = [
            cont for type_conts in self.trajectory_controllers.itervalues() for cont in type_conts]

    @staticmethod
    def _change_ctrl(controllers, managed_controllers):
        """
        Switch the current controller
        """
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

            switch_controllers = rospy.ServiceProxy(
                'controller_manager/switch_controller', SwitchController)
            try:
                resp1 = switch_controllers(
                    controllers_to_start, controllers_to_stop, SwitchControllerRequest.BEST_EFFORT, False, 0.0)
            except rospy.ServiceException:
                success = False

            if not resp1.ok:
                success = False

        if not success:
            rospy.logwarn(
                "Failed to change some of the controllers. This is normal if this is not a 5 finger hand.")

        return success

    def change_hand_ctrl(self, controller):
        return self._change_ctrl(self.hand_controllers[controller], self.managed_hand_controllers)

    def change_trajectory_ctrl(self, controller):
        return self._change_ctrl(self.trajectory_controllers[controller],
                                 self.managed_trajectory_controllers)

    def change_force_ctrl_type(self, chng_type_msg):
        """
        Calls the service (sr_hand_robot/change_control_type) that allows to tell the driver (sr_robot_lib)
        which type of force control has to be sent to the motor:
            - torque demand (sr_robot_msgs::ControlType::FORCE)
            - PWM (sr_robot_msgs::ControlType::PWM)
        it will block for time_to_reload_params secs to allow hand_controllers parameters to be updated
        """
        success = True
        for hand_id in self.robot_ids:
            change_control_type = rospy.ServiceProxy(
                'sr_hand_robot/' + hand_id + '/change_control_type', ChangeControlType)
            try:
                resp1 = change_control_type(chng_type_msg)
                if resp1.result.control_type != chng_type_msg.control_type:
                    success = False
            except rospy.ServiceException:
                success = False

        # Allow some time to reload parameters
        rospy.sleep(self.time_to_reload_params)

        if not success:
            rospy.logwarn("Failed to change the control type.")

        return success
