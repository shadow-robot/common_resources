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
import os
import requests
import json
import random
import string
import shutil
import rospy
import rospkg
from datetime import datetime
from sr_utilities_common.manual_test_suite import ManualTestSuite
from sr_utilities_common.aws_manager import AWS_Manager

BUCKET_NAME = "shadowtestbucket"
API_URL = "https://ddo2pew5xd.execute-api.eu-west-2.amazonaws.com/prod"
UPLOAD_PATH = "/tmp/upload"
DOWNLOAD_PATH = "/tmp/download"


class Test_AWS_Manager:
    def __init__(self):
        time = datetime.now()
        current_time = time.strftime("%d%m%Y_%H.%M.%S")
        self.filename = f"test_{current_time}"

        response = requests.get(API_URL)
        response_json = json.loads(response.text)
        aws_details = response_json["body"]
        self.aws_manager = AWS_Manager(access_key=aws_details["access_key"],
                                       secret_key=aws_details["secret_key"],
                                       session_token=aws_details["session_token"])

    def aws_upload(self):
        self.make_files_to_be_uploaded()
        self.aws_manager.upload(BUCKET_NAME, "/tmp", "/upload", [self.filename], None)
        folder_list = self.aws_manager.get_bucket_structure_with_prefix(BUCKET_NAME, None)
        if folder_list is not None:
            for element in folder_list:
                if element['Key'] == f"/upload/{self.filename}":
                    return True
        return False

    def aws_download(self):
        self.delete_download_folder()
        self.aws_manager.download(BUCKET_NAME, "/tmp", "download", [self.filename], "/upload")
        if os.path.exists(f"{DOWNLOAD_PATH}/{self.filename}"):
            return True
        return False
    
    def make_files_to_be_uploaded(self):
        if not os.path.exists(UPLOAD_PATH):
            os.mkdir(UPLOAD_PATH)
        with open(f"{UPLOAD_PATH}/{self.filename}", "w") as upload_file:
            letters = string.ascii_lowercase
            message = ''.join(random.choice(letters) for i in range(10))
            upload_file.write(message)

    def delete_download_folder(self):
        if os.path.exists(DOWNLOAD_PATH):
            shutil.rmtree(DOWNLOAD_PATH)


if __name__ == '__main__':
    rospy.init_node('aws_manager_test')

    test_aws_manager = Test_AWS_Manager()
    method_list = ['aws_upload', 'aws_download']
    test_suite = ManualTestSuite(test_aws_manager, method_list)
