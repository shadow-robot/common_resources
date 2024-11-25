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

def get_directory_size(directory: str) -> int:
    """
        Get the size of a directory in bytes. This function is recursive and will calculate the size of the direcotry
        and all its subdirectories.

        :param directory: The directory to calculate the size of.
        :return: The size of the directory in bytes.
    """

    if not os.path.exists(directory):
        raise FileNotFoundError(f"Directory '{directory}' does not exist.")
    
    if not os.path.isdir(directory):
        raise NotADirectoryError(f"Path '{directory}' is not a directory.")

    total_size = 0

    for root, _, files in os.walk(directory):
        for file in files:
            total_size += os.path.getsize(os.path.join(root, file))

    return total_size
