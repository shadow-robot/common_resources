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

import argparse
import os
import re
import requests
import sys


if __name__ == "__main__":
    default_customer_key_file_path = '/usr/local/bin/customer.key'
    parser = argparse.ArgumentParser(description="Enable the AWS CLI by using an API/Customer key to request access "
                                                 "keys and storing them in the user's AWS config files. Without "
                                                 "arguments, expects the API/Customer key to be in {}.".format(
                                                     default_customer_key_file_path
                                                 ))
    parser.add_argument("--api_key", help="The API/customer key to use when requesting access keys.")
    parser.add_argument("--api_key_file", help="The path to a file containing the API/customer key to use when "
                                               "requesting access keys.", default=default_customer_key_file_path)
    args = parser.parse_args()
    if args.api_key:
        customer_key = "{}\n".format(args.api_key)
    else:
        customer_key_file_path = os.path.expanduser(args.api_key_file)
        print("Reading API/customer key from: {}".format(customer_key_file_path))
        try:
            with open(customer_key_file_path, 'r') as customer_key_file:
                customer_key = customer_key_file.read()
        except:
            print("Could not read API/customer key from {}, ask software team for help!".format(customer_key_file_path))
            sys.exit(1)
    print("Using API/customer key: {}".format(customer_key))

    headers = {
        'x-api-key': '{}'.format(customer_key[:-1]),
    }

    try:
        response = requests.get('https://5vv2z6j3a7.execute-api.eu-west-2.amazonaws.com/prod', headers=headers)
    except:
        print("Couldn't request secret AWS access key, retry or ask software team for help!")

    result = re.search('ACCESS_KEY_ID=(.*)\nSECRET_ACCESS', response.text)
    if result is None:
        print("No access key in the response from Amazon's servers. Maybe you have a bad API/customer key?")
        sys.exit(1)
    aws_access_key_id = result.group(1)

    result = re.search('SECRET_ACCESS_KEY=(.*)\nSESSION_TOKEN', response.text)
    if result is None:
        print("No secret access key in the response from Amazon's servers. Maybe you have a bad API/customer key?")
        sys.exit(1)
    aws_secret_access_key = result.group(1)

    result = re.search('SESSION_TOKEN=(.*)\nEXPIRATION', response.text)
    if result is None:
        print("No session token in the response from Amazon's servers. Maybe you have a bad API/customer key?")
        sys.exit(1)
    aws_session_token = result.group(1)

    output = "[default]\naws_access_key_id={}\naws_secret_access_key={}\naws_session_token={}\n".format(
        aws_access_key_id, aws_secret_access_key, aws_session_token
    )

    aws_dir_path = os.path.expanduser("~/.aws")
    if os.path.exists(aws_dir_path):
        if os.path.isfile(aws_dir_path):
            print("{} exists but it is not a directory.".format(aws_dir_path))
            sys.exit(1)
    else:
        os.mkdir(aws_dir_path)
    aws_credentials_filename = "{}/credentials".format(aws_dir_path)
    try:
        with open(aws_credentials_filename, "w+") as aws_credentials_file:
            aws_credentials_file.write(output)
    except Exception as e:
        print("Failed to write to {}:\n{}".format(aws_credentials_filename, e))
    print("Successfully obtained AWS credentials. You can now use the AWS CLI.")
