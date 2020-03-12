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

import rospy
import logging
import boto3
from botocore.exceptions import ClientError
import subprocess
import requests
import re
import json
import os

if __name__ == "__main__":
    rospy.init_node("aws_manager_node")

    download_param = rospy.get_param("~download")
    upload_param = rospy.get_param("~upload")
    bucket_name = rospy.get_param("~bucket_name")
    files_base_path = rospy.get_param("~files_base_path")
    files_folder_path = rospy.get_param("~files_folder_path")
    file_names = rospy.get_param("~file_names")

    file_full_paths = []
    aws_paths = []

    for file_name in file_names:
        file_full_paths.append("{}/{}/{}".format(files_base_path, files_folder_path, file_name))
        aws_paths.append("{}/{}".format(files_folder_path, file_name))

    try:
        with open('/usr/local/bin/customer.key', 'r') as customer_key_file:
            customer_key = customer_key_file.read()
    except:
        rospy.logerr("Could not find customer key, ask software team for help!")

    headers = {
        'x-api-key': '{}'.format(customer_key[:-1]),
    }

    try:
        response = requests.get('https://5vv2z6j3a7.execute-api.eu-west-2.amazonaws.com/prod', headers=headers)
    except:
        rospy.logerr("Could request secret AWS access key, ask software team for help!")

    result = re.search('ACCESS_KEY_ID=(.*)\nSECRET_ACCESS', response.text)
    aws_access_key_id = result.group(1)

    result = re.search('SECRET_ACCESS_KEY=(.*)\nSESSION_TOKEN', response.text)
    aws_secret_access_key = result.group(1)

    result = re.search('SESSION_TOKEN=(.*)\nEXPIRATION', response.text)
    aws_session_token = result.group(1)

    client = boto3.client(
        's3',
        aws_access_key_id=aws_access_key_id,
        aws_secret_access_key=aws_secret_access_key,
        aws_session_token=aws_session_token
    )

    if upload_param is True:
        for file_full_path, aws_path in zip(file_full_paths, aws_paths):
            rospy.loginfo("Uploading {} file..".format(file_full_path))
            client.upload_file(file_full_path, bucket_name, aws_path)
            rospy.loginfo("Completed file upload.")

    if download_param is True:
        directory = "{}/{}".format(files_base_path, files_folder_path)
        if not os.path.exists(directory):
            os.makedirs(directory)

        for file_full_path, aws_path in zip(file_full_paths, aws_paths):
            rospy.loginfo("Downloading {} file.".format(aws_path))
            client.download_file(bucket_name, aws_path, file_full_path)
            rospy.loginfo("Completed file download.")

    rospy.signal_shutdown("")
