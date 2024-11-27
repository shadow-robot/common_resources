#!/usr/bin/env python3

# Copyright 2024 Shadow Robot Company Ltd.
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
import shutil
import argparse
import logging_utils as utils

PROTECTED_ITEMS = ['latest', 'core_dumps']

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=("Remove logs until the size of the logs is less than" +
                                                  "a specified size."))
    parser.add_argument('-m', '--min_size_of_logs', type=int, default=utils.GIGABYTE,
                        help="The minimum size of the logs in bytes. Default is 1GB.")
    args = parser.parse_args()

    min_size_of_logs = args.min_size_of_logs

    print(f"Removing logs until the size of the logs is less than {min_size_of_logs} bytes.")

    current_log_size = utils.get_directory_size(utils.LOG_PATH)
    core_dumps_exists = os.path.exists(utils.CORE_DUMPS_PATH)

    while current_log_size > min_size_of_logs:
        oldest_in_log, oldest_in_log_ctime = utils.get_oldest_item_in_directory(utils.LOG_PATH,
                                                                                search_blacklist=PROTECTED_ITEMS)

        if core_dumps_exists:
            oldest_in_core_dumps, oldest_in_core_dumps_ctime = utils.get_oldest_item_in_directory(utils.CORE_DUMPS_PATH)

            if oldest_in_log_ctime <= oldest_in_core_dumps_ctime:
                oldest_abs_path = os.path.join(utils.LOG_PATH, oldest_in_log)
            else:
                oldest_abs_path = os.path.join(utils.CORE_DUMPS_PATH, oldest_in_core_dumps)
        else:
            oldest_abs_path = os.path.join(utils.LOG_PATH, oldest_in_log)

        print(f"\t- {oldest_abs_path}")

        if os.path.isdir(oldest_abs_path):
            current_log_size -= utils.get_directory_size(oldest_abs_path)

            # Use shutil.rmtree to remove directories. os.rmdir will fail if the directory is not empty.
            # shutil.rmtree will fail if a directory contains read-only files. ignore_errors=True will prevent this.
            shutil.rmtree(oldest_abs_path, ignore_errors=True)
        else:
            current_log_size -= os.path.getsize(oldest_abs_path)
            os.remove(oldest_abs_path)
