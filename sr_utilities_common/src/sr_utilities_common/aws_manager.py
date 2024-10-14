#!/usr/bin/env python3

# Copyright 2020-2022, 2024 Shadow Robot Company Ltd.
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
import re
import os
import sys
import time
import rospy
import botocore
import boto3
import boto3.s3.transfer as s3transfer
from six.moves import input
from tqdm import tqdm
import requests


class AWSManager:
    NUM_RETRIES = 5
    NUM_WORKERS = 20

    def __init__(self, access_key=None, secret_key=None, session_token=None):
        self.file_full_paths = []
        self.aws_paths = []
        aws_access_key_id = access_key
        aws_secret_access_key = secret_key
        aws_session_token = session_token
        if not access_key or not secret_key or not session_token:
            self._aws_access_key_id, self._aws_secret_access_key, self._aws_session_token = self._get_auth_keys()
        else:
            self._aws_access_key_id = aws_access_key_id
            self._aws_secret_access_key = aws_secret_access_key
            self._aws_session_token = aws_session_token
        botocore_config = botocore.config.Config(max_pool_connections=self.NUM_WORKERS)
        self._client = boto3.client(
            's3',
            aws_access_key_id=self._aws_access_key_id,
            aws_secret_access_key=self._aws_secret_access_key,
            aws_session_token=self._aws_session_token,
            config=botocore_config
        )

    def _get_auth_keys(self):
        try:
            with open('/usr/local/bin/customer.key', 'r', encoding="utf-8") as customer_key_file:
                customer_key = customer_key_file.read()
                headers = {'x-api-key': f'{customer_key[:-1]}'}
        except IOError:
            rospy.logerr("Could not find customer key, ask software team for help!")
        try:
            for attempt_number in range(self.NUM_RETRIES):
                response = requests.get('https://5vv2z6j3a7.execute-api.eu-west-2.amazonaws.com/prod',
                                        headers=headers)
                if response.status_code == 200:
                    break
                rospy.logwarn(f"AWS S3 authentication returned response status code {response.status_code}, " +
                              f"retrying... (attempt {attempt_number + 1}/{self.NUM_RETRIES})")
                time.sleep(0.2)
            if response.status_code != 200:
                rospy.logerr(f"Could not connect to AWS API server. Returned status code {response.status_code}")
                if response.status_code == 502:
                    rospy.logerr("502 is a load balancer error and normally doesn't last long. Please try again")
                response.raise_for_status()
            result = re.search('ACCESS_KEY_ID=(.*)\nSECRET_ACCESS', response.text)
            aws_access_key_id = result.group(1)
            result = re.search('SECRET_ACCESS_KEY=(.*)\nSESSION_TOKEN', response.text)
            aws_secret_access_key = result.group(1)
            result = re.search('SESSION_TOKEN=(.*)\nEXPIRATION', response.text)
            aws_session_token = result.group(1)
        except requests.exceptions.RequestException as exception:
            err = "Could not request secret AWS access key, ask software team for help!"
            err = err + f"\nError message: {exception}"
            rospy.logerr(err)
        return aws_access_key_id, aws_secret_access_key, aws_session_token

    @property
    def aws_credentials(self):
        creds = {
            "aws_access_key_id": self._aws_access_key_id,
            "aws_secret_access_key": self._aws_secret_access_key,
            "aws_session_token": self._aws_session_token
        }
        return creds

    def get_bucket_structure_with_prefix(self, bucket_name, prefix=""):
        try:
            command = self._client.list_objects(Bucket=bucket_name)
            if prefix:
                command = self._client.list_objects(Bucket=bucket_name, Prefix=prefix)
            if 'Contents' in command:
                return command['Contents']
        except self._client.exceptions.NoSuchBucket as exception:
            rospy.logerr(f"Failed listing bucket objects. {exception}")
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
            if content['Key'][-1] == "/":
                continue
            if aws_subfolder:
                filenames.append(content['Key'][len(aws_subfolder):])
            else:
                filenames.append(content['Key'])
        return filenames

    def _prepare_structure_upload(self, files_base_path, files_folder_path, file_names, bucket_subfolder):
        self.file_full_paths = []
        self.aws_paths = []
        for file_name in file_names:
            self.file_full_paths.append(f"{files_base_path}/{files_folder_path}/{file_name}")
            if bucket_subfolder:
                self.aws_paths.append(f"{bucket_subfolder}/{file_name}")
            else:
                self.aws_paths.append(f"{files_folder_path}/{file_name}")

    def _prepare_structure_download(self, files_base_path, files_folder_path, file_names, bucket_subfolder):
        self.file_full_paths = []
        self.aws_paths = []
        for file_name in file_names:
            self.file_full_paths.append(f"{files_base_path}/{files_folder_path}/{file_name}")
            if bucket_subfolder:
                self.aws_paths.append(f"{bucket_subfolder}/{file_name}")
            else:
                self.aws_paths.append(f"{file_name}")

    def download(self, bucket_name, files_base_path, files_folder_path,
                 file_names, bucket_subfolder, preserve_downloaded_folder_structure=False,
                 file_extentions_to_include=None):
        if file_extentions_to_include is not None:
            examples = "(e.g. ['.txt', '.jpg'])"
            if not isinstance(file_extentions_to_include, list):
                raise ValueError(f"file_extentions_to_include must be a list of extensions {examples}")
            for file_extention in file_extentions_to_include:
                if not file_extention.startswith('.'):
                    raise ValueError(f"each file_extentions_to_include item must start with a '.' {examples}")
                if not isinstance(file_extention, str):
                    raise ValueError("file_extentions_to_include must be a list of strings")
        self._prepare_structure_download(files_base_path, files_folder_path, file_names, bucket_subfolder)
        download_succeded = False
        directory = os.path.join(files_base_path, files_folder_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
        for file_full_path, aws_path in tqdm(zip(self.file_full_paths, self.aws_paths),
                                             total=len(self.file_full_paths)):
            if preserve_downloaded_folder_structure:
                if not os.path.exists(os.path.join(directory, aws_path)):
                    os.makedirs(os.path.join(directory, aws_path))
                file_full_path = os.path.join(files_base_path, files_folder_path, aws_path)
                if os.path.isdir(file_full_path):
                    os.rmdir(file_full_path)
            if file_extentions_to_include is not None:
                if not any(file_full_path.endswith(ext) for ext in file_extentions_to_include):
                    ext = file_full_path.split('.')[-1]
                    rospy.loginfo(f"Skipping file download ({aws_path}) due to {ext} not being " +
                                  "in file_extentions_to_include")
                    continue
            try:
                object_size = self._client.head_object(Bucket=bucket_name, Key=aws_path)['ContentLength']
                with tqdm(total=object_size,
                          unit="B", unit_scale=True,
                          desc=file_full_path.replace(directory, ''),
                          leave=False) as pbar:
                    self._client.download_file(
                        Bucket=bucket_name,
                        Key=aws_path,
                        Filename=file_full_path,
                        Callback=pbar.update,  # pylint: disable=W0640,W0108
                    )
                download_succeded = True
            except self._client.exceptions.ClientError as exception:
                rospy.logerr(f"File download failed ({aws_path}). {exception}")
        return download_succeded

    def parallel_upload(self, bucketname, s3dir, filelist, progress_func):
        transfer_config = s3transfer.TransferConfig(
            use_threads=True,
            max_concurrency=self.NUM_WORKERS,
        )
        s3t = s3transfer.create_transfer_manager(self._client, transfer_config)
        futures = []
        success = True
        for src, dst in zip(filelist, s3dir):
            futures.append(s3t.upload(src, bucketname, dst,
                                      subscribers=[s3transfer.ProgressCallbackInvoker(progress_func)]))
        s3t.shutdown()  # wait for all the upload tasks to finish
        success = True
        try:
            for future in futures:
                future.result()
        except Exception as err:
            rospy.logerr(f"Error uploading file to S3: {err}")
            success = False
        return success

    def upload(self, bucket_name, files_base_path, files_folder_path, file_names, bucket_subfolder,
               strip_root_dir=False, fast_upload=True):
        self._prepare_structure_upload(files_base_path, files_folder_path, file_names, bucket_subfolder)
        if strip_root_dir:
            for remote_path_index in range(len(self.aws_paths)):
                self.aws_paths[remote_path_index] = self.aws_paths[remote_path_index].split("/", 1)[1]
        if fast_upload:
            total_size = sum(os.path.getsize(file_full_path) for file_full_path in self.file_full_paths)
            with tqdm(total=total_size, desc="Uploading files to S3", unit='B', unit_scale=1) as pbar:
                upload_succeded = self.parallel_upload(bucket_name,
                                                       self.aws_paths,
                                                       self.file_full_paths,
                                                       pbar.update)
        else:
            upload_succeded = False
            for file_full_path, aws_path in tqdm(zip(self.file_full_paths, self.aws_paths),
                                                 total=len(self.file_full_paths),
                                                 desc="Uploading files to S3"):
                try:
                    self._client.upload_file(file_full_path, bucket_name, aws_path)
                    upload_succeded = True
                except self._client.exceptions.S3UploadFailedError as exception:
                    rospy.logerr(f"File upload failed ({file_full_path}). {exception}")
        return upload_succeded


def gather_all_files_local(files_base_path, files_folder_path):
    path_string = f"{files_base_path}/{files_folder_path}"
    file_names = []
    for path, _, files in os.walk(path_string):
        for file_name in files:
            # We only want the file name, not bucket subfolder.
            file_names.append(f"{path}/{file_name}"[len(path_string)+1:])
    return file_names


def validated_files_to_be_downloaded(bucket_name, files_base_path, files_folder_path,
                                     file_names, bucket_subfolder, preserve_downloaded_folder_structure=False):
    print_msg = f"\nFrom bucket {bucket_name} downloading the files:"
    print_msg_2 = ""
    for filename in file_names:
        if preserve_downloaded_folder_structure and bucket_subfolder:
            print_msg_2 += f"\n    {files_base_path}/{files_folder_path}/{bucket_subfolder}/{filename}"
        else:
            print_msg_2 += f"\n    {files_base_path}/{files_folder_path}/{filename}"
        if bucket_subfolder:
            print_msg += f"\n    {bucket_subfolder}/{filename}"
        else:
            print_msg += f"\n    {filename}"
    print_msg += f"\n\nThese files will be downloaded to {files_base_path}/{files_folder_path}:"
    print_msg += print_msg_2
    rospy.loginfo(print_msg)
    value = input("Would you like to download these files? (y/n) ")
    value = value.lower().strip()

    if value == "y":
        return True
    if value == "n":
        return False

    rospy.logerr("Select a valid option")
    validated_files_to_be_downloaded(bucket_name, files_base_path, files_folder_path,
                                     file_names, bucket_subfolder, preserve_downloaded_folder_structure)
    return True


def validated_files_to_be_uploaded(bucket_name, files_base_path, files_folder_path,
                                   file_names, bucket_subfolder):
    print_msg = f"\nFrom {files_base_path}/{files_folder_path} uploading the files:"
    print_msg_2 = ""
    for filename in file_names:
        print_msg += f"\n    {files_folder_path}/{filename}"
        if bucket_subfolder:
            print_msg_2 += f"\n    {bucket_subfolder}/{filename}"
        else:
            print_msg_2 += f"\n    {filename}"
    print_msg += f"\n\nThese files will be uploaded to the bucket {bucket_name}:"
    print_msg += print_msg_2
    rospy.loginfo(print_msg)
    value = input("Would you like to upload these files? (y/n) ")
    value = value.lower().strip()

    if value == "y":
        return True
    if value == "n":
        return False

    rospy.logerr("Select a valid option")
    validated_files_to_be_uploaded(bucket_name, files_base_path, files_folder_path,
                                   file_names, bucket_subfolder)
    return True


def return_function_mode(function_mode):
    if function_mode.lower().strip() not in ["download", "upload"]:
        err_message = "Please specify either upload or download to use this script"
        err_message += "\nE.g: function_mode:='upload'"
        err_message += "\n     function_mode:='download'"
        err_message += f"You entered: {function_mode}"
        rospy.logerr(err_message)
        rospy.signal_shutdown("")
        sys.exit(1)
    return function_mode


if __name__ == "__main__":
    rospy.init_node("aws_manager_node")

    function_mode_param = rospy.get_param("~function_mode")
    preserve_downloaded_folder_structure_param = rospy.get_param("~preserve_downloaded_folder_structure")
    skip_check_param = rospy.get_param("~skip_check")
    bucket_name_param = rospy.get_param("~bucket_name")
    bucket_subfolder_param = rospy.get_param("~bucket_subfolder")
    files_base_path_param = rospy.get_param("~files_base_path")
    files_folder_path_param = rospy.get_param("~files_folder_path")
    file_names_param = rospy.get_param("~file_names")
    file_extentions_to_include_param = rospy.get_param("~file_extentions_to_include", None)
    if file_extentions_to_include_param is not None:
        file_extentions_to_include_param = list(file_extentions_to_include_param)

    function_mode_param = return_function_mode(function_mode_param)
    if bucket_subfolder_param == "":
        bucket_subfolder_param = None
    aws_manager = AWSManager()

    if function_mode_param == "upload":
        if preserve_downloaded_folder_structure_param:
            rospy.logwarn("preserve_downloaded_folder_structure is not supported for upload")
        if file_names_param == "":
            file_names_param = gather_all_files_local(files_base_path_param, files_folder_path_param)
        if not skip_check_param:
            if not validated_files_to_be_uploaded(bucket_name_param, files_base_path_param, files_folder_path_param,
                                                  file_names_param, bucket_subfolder_param):
                rospy.signal_shutdown("")
                sys.exit(0)
        status_msg = "File upload failed"
        if aws_manager.upload(bucket_name_param, files_base_path_param, files_folder_path_param,
                              file_names_param, bucket_subfolder_param):
            status_msg = "Completed file upload."
        rospy.loginfo(status_msg)

    if function_mode_param == "download":
        if file_names_param == "":
            file_names_param = aws_manager.gather_all_files_remote(bucket_name_param, bucket_subfolder_param)
        if not skip_check_param:
            if not validated_files_to_be_downloaded(bucket_name_param, files_base_path_param, files_folder_path_param,
                                                    file_names_param, bucket_subfolder_param,
                                                    preserve_downloaded_folder_structure_param):
                rospy.signal_shutdown("")
                sys.exit(0)
        status_msg = "File download failed"
        if aws_manager.download(bucket_name_param, files_base_path_param, files_folder_path_param,
                                file_names_param, bucket_subfolder_param, preserve_downloaded_folder_structure_param,
                                file_extentions_to_include_param):
            status_msg = "Completed file download."
        rospy.loginfo(status_msg)

    rospy.signal_shutdown("")
    sys.exit(0)
