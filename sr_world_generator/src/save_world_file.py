#!/usr/bin/env python

import re
import os
import sys
import rospy
import rospkg
import argparse
import subprocess
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

class GazeboWorldSaver(object):
    def __init__(self):

        self.model_and_pose = {}
        self.gazebo_generated_world_file_path = rospy.get_param('~gazebo_generated_world_file_path')
        self.output_world_file = rospy.get_param('~output_world_file_name', 'new_world') + '.world'
        self.description_path = rospkg.RosPack().get_path('sr_description_common')
        self.config_path = rospkg.RosPack().get_path('sr_world_generator') + '/config'

        self._check_if_world_file_exists()
        self._start_gazebo_with_newly_created_world()
        self._get_gazebo_models_states()
        self._extract_model_data_from_msg()
        self._initiate_world_file()
        self._save_lighting_config_to_world_file()
        self._save_models_to_world_file()
        self._save_physics_config_to_world_file()
        self._finish_up_world_file()
        self._clean_exit()

    def _check_if_world_file_exists(self):
        if not os.path.isfile(self.gazebo_generated_world_file_path):
            raise IOError("Gazebo generated world file does not exist!")

    def _start_gazebo_with_newly_created_world(self):
        self.process = subprocess.Popen(['xterm -e roslaunch sr_world_generator create_world_template.launch gui:=false \
                                        scene:=true world:={}'.format(self.gazebo_generated_world_file_path)],
                                        shell=True)

    def _clean_exit(self):
        rospy.loginfo("World saved!")
        self.process.kill()
        rospy.sleep(3)
        sys.exit(0)

    def _get_gazebo_models_states(self):
        self.gazebo_model_states_msg = rospy.wait_for_message('/gazebo/model_states', ModelStates)

    def _extract_model_data_from_msg(self):
        for model_name, pose in zip(self.gazebo_model_states_msg.name, self.gazebo_model_states_msg.pose):
            if 'ursrbox' == model_name:
                continue
            position_as_list = [pose.position.x, pose.position.y, pose.position.z]
            orientation_as_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            orientation_as_list_euler = list(euler_from_quaternion(orientation_as_list))
            pose_as_list = position_as_list + orientation_as_list_euler
            self.model_and_pose[model_name] = ' '.join(str(val) for val in pose_as_list)

    def _save_models_to_world_file(self):
        all_objects_string = ''
        for key, value in self.model_and_pose.iteritems():
            all_objects_string += '    <include>\n'
            all_objects_string += '      <uri>model://' + re.sub(r'_\d+$', '', key) + '</uri>\n'
            all_objects_string += '      <static>true</static>\n'
            all_objects_string += '      <name>' + key + '</name>\n'
            all_objects_string += '      <pose>' + value + '</pose>\n'
            all_objects_string += '    </include>\n'
        self._save_to_world_file(all_objects_string)

    def _save_lighting_config_to_world_file(self):
        with open (self.config_path + '/gazebo_light_string', 'r') as myfile:
            data = myfile.readlines()
        data = ''.join(data) + '\n'
        self._save_to_world_file(data)

    def _save_physics_config_to_world_file(self):
        with open (self.config_path + '/gazebo_physics_string', 'r') as myfile:
            data = myfile.readlines()
        data = ''.join(data) + '\n'
        self._save_to_world_file(data)

    def _initiate_world_file(self):
        self._remove_output_file_if_exists()
        leading_string = '<?xml version="1.0" ?>\n' + '<sdf version="1.4">\n' + '  <world name="default">\n'
        self._save_to_world_file(leading_string)

    def _finish_up_world_file(self):
        trailing_string = '  </world>\n' +  '</sdf>'
        self._save_to_world_file(trailing_string)

    def _save_to_world_file(self, string):
        with open(self.description_path + '/worlds/' + self.output_world_file, 'a') as myfile:
            myfile.write(string)

    def _remove_output_file_if_exists(self):
        full_file_path = self.description_path + '/worlds/' + self.output_world_file
        if os.path.isfile(full_file_path):
            os.remove(full_file_path)

if __name__ == '__main__':
    rospy.init_node('save_gazebo_world_file', anonymous=True)
    gws = GazeboWorldSaver()
