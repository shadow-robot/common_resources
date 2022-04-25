#!/usr/bin/python3

# Copyright 2021-2022 Shadow Robot Company Ltd.
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

# pylint: disable=W1618
# TODO REMOVE ABOVE WHEN USING NEW LINT
import os
import requests
import json
import random
import string
import shutil
import rospy
import rostest
from datetime import datetime
from unittest import TestCase
from sr_utilities_common.aws_manager import AWS_Manager

BUCKET_NAME = "shadowtestbucket"
API_URL = "https://ddo2pew5xd.execute-api.eu-west-2.amazonaws.com/prod"
UPLOAD_PATH = "/tmp/upload"
DOWNLOAD_PATH = "/tmp/download"


class TestAWSManager(TestCase):
    @classmethod
    def setUpClass(self):
        rospy.logerr("AHHHHH")
        response = requests.get(API_URL)
        response_json = json.loads(response.text)
        aws_details = response_json["body"]
        self.aws_manager = AWS_Manager(access_key=aws_details["access_key"],
                                       secret_key=aws_details["secret_key"],
                                       session_token=aws_details["session_token"])
        self.filename = self.get_filename()

    @classmethod
    def tearDownClass(self):
        pass

    def test_aws_01_upload(self):
        result = False
        self.make_files_to_be_uploaded()
        self.aws_manager.upload(BUCKET_NAME, "/tmp", "upload", [self.filename], None)
        folder_list = self.aws_manager.get_bucket_structure_with_prefix(BUCKET_NAME, None)
        if folder_list is not None:
            for element in folder_list:
                if element['Key'] == f"upload/{self.filename}":
                    result = True
        self.assertTrue(result)

    def test_aws_02_upload_subfolder(self):
        self.filename = self.get_filename()  # Second file should have new filename
        result = False
        self.make_files_to_be_uploaded()
        self.aws_manager.upload(BUCKET_NAME, "/tmp", "upload", [self.filename], "Subfolder")
        folder_list = self.aws_manager.get_bucket_structure_with_prefix(BUCKET_NAME, None)
        if folder_list is not None:
            for element in folder_list:
                if element['Key'] == f"Subfolder/{self.filename}":
                    result = True
        self.assertTrue(result)

    def test_aws_03_download(self):
        result = False
        self.delete_download_folder()
        self.aws_manager.download(BUCKET_NAME, "/tmp", "download", [self.filename], "upload")
        if os.path.exists(f"{DOWNLOAD_PATH}/{self.filename}"):
            result = True
        self.assertTrue(result)

    def test_aws_04_download_subfolder(self):
        result = False
        self.aws_manager.download(BUCKET_NAME, "/tmp", "download", [self.filename], "Subfolder")
        if os.path.exists(f"{DOWNLOAD_PATH}/{self.filename}"):
            result = True
        self.assertTrue(result)

    def test_aws_05_remove_all_files(self):
        rospy.sleep(4)
        folder_list = self.aws_manager.get_bucket_structure_with_prefix(BUCKET_NAME, None)
        if folder_list is not None:
            for element in folder_list:
                filepath = element['Key']
                rospy.loginfo(filepath)
                self.aws_manager._client.delete_object(Bucket=BUCKET_NAME, Key=filepath)
        # Check bucket status after removing everything.
        folder_list = self.aws_manager.get_bucket_structure_with_prefix(BUCKET_NAME, None)
        if not folder_list:
            result = True
        else:
            result = False
        self.assertTrue(result)

    def make_files_to_be_uploaded(self):
        if not os.path.exists(UPLOAD_PATH):
            os.mkdir(UPLOAD_PATH)
        with open(f"{UPLOAD_PATH}/{self.filename}", "w") as upload_file:
            letters = string.ascii_lowercase
            message = ''.join(random.choice(letters) for i in range(10))
            upload_file.write(message)

    def get_filename(self):
        time = datetime.now()
        current_time = time.strftime("%d%m%Y_%H.%M.%S")
        return f"test_{current_time}"

    def delete_download_folder(self):
        if os.path.exists(DOWNLOAD_PATH):
            shutil.rmtree(DOWNLOAD_PATH)


if __name__ == '__main__':
    PKGNAME = 'sr_utilities_common'
    NODENAME = 'aws_manager_test'
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, TestAWSManager)
    os._exit(0)
