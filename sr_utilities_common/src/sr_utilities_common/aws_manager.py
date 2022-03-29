#!/usr/bin/env python3

# Copyright 2020-2022 Shadow Robot Company Ltd.
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
import rospy
import logging
import boto3
from botocore.exceptions import *
import subprocess
import requests
import re
import json
import os

class AWS_Manager(object):
    def __init__(self):
        self.file_full_paths = []
        self.aws_paths = []
        aws_access_key_id = ""
        aws_secret_access_key = ""
        aws_session_token = ""
        headers = None

        try:
            with open('/usr/local/bin/customer.key', 'r') as customer_key_file:
                customer_key = customer_key_file.read()
                headers = {'x-api-key': '{}'.format(customer_key[:-1])}
        except Exception:
            rospy.logerr("Could not find customer key, ask software team for help!")

        try:
            response = requests.get('https://5vv2z6j3a7.execute-api.eu-west-2.amazonaws.com/prod', headers=headers)

            if response.status_code != 200:  # Code for success
                raise Exception()

            result = re.search('ACCESS_KEY_ID=(.*)\nSECRET_ACCESS', response.text)
            aws_access_key_id = result.group(1)
            result = re.search('SECRET_ACCESS_KEY=(.*)\nSESSION_TOKEN', response.text)
            aws_secret_access_key = result.group(1)
            result = re.search('SESSION_TOKEN=(.*)\nEXPIRATION', response.text)
            aws_session_token = result.group(1)

        except Exception:
            rospy.logerr("Could not request secret AWS access key, ask software team for help!")

        self._client = boto3.client(
            's3',
            aws_access_key_id=aws_access_key_id,
            aws_secret_access_key=aws_secret_access_key,
            aws_session_token=aws_session_token
        )

    def get_bucket_structure_with_prefix(self, bucket_name, prefix):
        if prefix:
            try:
                if 'Contents' in self._client.list_objects(Bucket=bucket_name, Prefix=prefix):
                    return self._client.list_objects(Bucket=bucket_name, Prefix=prefix)['Contents']
            except Exception as e:
                rospy.logwarn("Failed listing bucket objects. " + str(e))
        return None

    def gather_all_files_remote(self, aws_bucket, aws_subfolder=None):
        if aws_subfolder:
            if aws_subfolder[-1] != "/":
                aws_subfolder += "/"  # Needed for formatting
            bucket_data = self._client.list_objects(Bucket=aws_bucket, Prefix=aws_subfolder)
        else:
            bucket_data = self._client.list_objects(Bucket=aws_bucket)
        
        filenames = []
        for content in bucket_data['Contents']:
            if aws_subfolder:
                filenames.append(content['Key'][len(aws_subfolder):])
            else:
                filenames.append(content['Key'])

        return filenames

    def _prepare_structure_upload(self, files_base_path, files_folder_path,
        file_names, bucket_subfolder=None):
        self.file_full_paths = []
        self.aws_paths = []
        for file_name in file_names:
            self.file_full_paths.append("{}/{}/{}".format(files_base_path, files_folder_path, file_name))
            if bucket_subfolder:
                self.aws_paths.append(f"{bucket_subfolder}/{files_folder_path}/{file_name}")
            else:
                self.aws_paths.append(f"{files_folder_path}/{file_name}")
    
    def _prepare_structure_download(self, files_base_path, files_folder_path,
        file_names, bucket_subfolder=None):
        self.file_full_paths = []
        self.aws_paths = []
        for file_name in file_names:
            self.file_full_paths.append("{}/{}/{}".format(files_base_path, files_folder_path, file_name))
            if bucket_subfolder:
                self.aws_paths.append(f"{bucket_subfolder}/{file_name}")
            else:
                self.aws_paths.append(file_name)

    def download(self, bucket_name, files_base_path, files_folder_path,
        file_names, bucket_subfolder=None):
        self._prepare_structure_download(files_base_path, files_folder_path, file_names, bucket_subfolder)
        downloadSucceded = False
        for file_full_path, aws_path in zip(self.file_full_paths, self.aws_paths):
            try:
                directory = os.path.dirname(file_full_path)
                if not os.path.exists(directory):
                    os.makedirs(directory)
                rospy.loginfo(f"{directory} \n{file_full_path}")
                self._client.download_file(bucket_name, aws_path, file_full_path)
                downloadSucceded = True
            except Exception as e:
                rospy.logwarn("File download failed. " + str(e))
        return downloadSucceded

    def upload(self, bucket_name, files_base_path, files_folder_path,
        file_names, bucket_subfolder=None):
        self._prepare_structure_upload(files_base_path, files_folder_path, file_names, bucket_subfolder)
        uploadSucceded = False
        for file_full_path, aws_path in zip(self.file_full_paths, self.aws_paths):
            try:
                self._client.upload_file(file_full_path, bucket_name, aws_path)
                uploadSucceded = True
            except Exception as e:
                rospy.loginfo("File upload failed" + str(e))
        return uploadSucceded
    
def gather_all_files_local(files_base_path, files_folder_path):
    path_string = f"{files_base_path}/{files_folder_path}"
    file_names = []
    for path, subdirs, files in os.walk(path_string):
        for f in files:
            # We only want the file name, not bucket subfolder.
            file_names.append(f"{path}/{f}"[len(path_string)+1:])
    return file_names


if __name__ == "__main__":
    rospy.init_node("aws_manager_node")

    download_param = rospy.get_param("~download")
    upload_param = rospy.get_param("~upload")
    bucket_name = rospy.get_param("~bucket_name")
    bucket_subfolder = rospy.get_param("~bucket_subfolder")
    files_base_path = rospy.get_param("~files_base_path")
    files_folder_path = rospy.get_param("~files_folder_path")
    file_names = rospy.get_param("~file_names")

    if bucket_subfolder == "":
        bucket_subfolder = None

    aws_manager = AWS_Manager()

    if upload_param is True:
        if file_names == "":
            file_names = gather_all_files_local(files_base_path, files_folder_path)
        status_msg = "File upload failed"
        rospy.loginfo("Uploading {}/{} file.".format(files_folder_path, file_names))
        if aws_manager.upload(bucket_name, files_base_path, files_folder_path,
            file_names, bucket_subfolder):
            status_msg = "Completed file upload."
        rospy.loginfo(status_msg)

    if download_param is True:
        if file_names == "":
            file_names = aws_manager.gather_all_files_remote(bucket_name, bucket_subfolder)
        status_msg = "File download failed"
        rospy.loginfo("Downloading {}/{} file.".format(files_folder_path, file_names))
        if aws_manager.download(bucket_name, files_base_path, files_folder_path,
            file_names, bucket_subfolder):
            status_msg = "Completed file download."
        rospy.loginfo(status_msg)

    rospy.signal_shutdown("")
