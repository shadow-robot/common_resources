#!/usr/bin/env python

import re
import rospy
import argparse
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates

class GazeboWorldSaver(object):
    def __init__(self):
        self.model_and_pose = {}
        model_states_msg = self._get_gazebo_models_states()
        self._extract_model_data_from_msg(model_states_msg)
    
        self._initiate_world_file()
        self._save_lighting_config_to_world_file()
        self._save_models_to_world_file()
        self._save_physics_config_to_world_file()
        self._finish_up_world_file()

    def _get_gazebo_models_states(self):
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        return model_states

    def _extract_model_data_from_msg(self, gazebo_model_states_msg):
        for model_name, pose in zip(gazebo_model_states_msg.name, gazebo_model_states_msg.pose):
            if 'ursrbox' == model_name:
                continue
            position_as_list = [pose.position.x, pose.position.y, pose.position.z]
            orientation_as_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            orientation_as_list_euler = list(euler_from_quaternion(orientation_as_list))
            pose_as_list = position_as_list + orientation_as_list_euler
            self.model_and_pose[model_name] = ' '.join(str(val) for val in pose_as_list)

    def _save_models_to_world_file(self):
        with open("test.world", "a") as myfile:
            for key, value in self.model_and_pose.iteritems():
                myfile.write('    <include>\n')
                myfile.write('      <uri>model://' + re.sub(r'_\d+$', '', key) + '</uri>\n')
                myfile.write('      <static>true</static>\n')
                myfile.write('      <name>' + key + '</name>\n')
                myfile.write('      <pose>' + value + '</pose>\n')
                myfile.write('    </include>\n')

    def _save_lighting_config_to_world_file(self):
        with open ("../config/gazebo_light_string", "r") as myfile:
            data = myfile.readlines()
        data = ''.join(data) + '\n'
        self._save_to_world_file(data)

    def _save_physics_config_to_world_file(self):
        with open ("../config/gazebo_physics_string", "r") as myfile:
            data = myfile.readlines()
        data = ''.join(data) + '\n'
        self._save_to_world_file(data)

    def _initiate_world_file(self):
        leading_string = '<?xml version="1.0" ?>\n' + '<sdf version="1.4">\n' + '  <world name="default">\n'
        self._save_to_world_file(leading_string)

    def _finish_up_world_file(self):
        trailing_string = '  </world>\n' +  '</sdf>'
        self._save_to_world_file(trailing_string)

    def _save_to_world_file(self, string):
        with open("test.world", "a") as myfile:
            myfile.write(string)

if __name__ == '__main__':
    rospy.init_node('serfow_state_machine', anonymous=True)
    gws = GazeboWorldSaver()
