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
from typing import Tuple

# CONSTANTS
GIGABYTE = 1024**3
LOG_PATH = os.path.join(os.path.expanduser('~'), '.ros', 'log')
CORE_DUMPS_PATH = os.path.join(LOG_PATH, 'core_dumps')


def get_directory_size(directory: str) -> int:
    """
        Get the size of a directory in bytes. This function is recursive and will calculate the size of the direcotry
        and all its subdirectories.

        :param directory: The absolute path of the directory to calculate the size of.
        :return: The size of the directory in bytes.
    """

    if not os.path.exists(directory):
        raise FileNotFoundError(f"Directory '{directory}' does not exist.")

    if not os.path.isdir(directory):
        raise NotADirectoryError(f"Path '{directory}' is not a directory.")

    total_size = 0

    for root, _, files in os.walk(directory):
        for _file in files:
            if os.path.islink(os.path.join(root, _file)):  # Ignore symbolic links
                continue
            total_size += os.path.getsize(os.path.join(root, _file))

    return total_size


def get_oldest_item_in_directory(directory: str, search_blacklist: list = None) -> Tuple[str, float]:
    """
        Get the oldest file/subdirectory in a directory based on the creation time.

        :param directory: The absolute path of the directory to search for the oldest file/subdirectory in.
        :param search_blacklist: A list of file/subdirectory names to ignore when searching for the oldest
            file/subdirectory. Optional.
        :return: A tuple containing the name of the oldest file/subdirectory and the creation time of the oldest
            file/subdirectory. If the directory is empty, returns None, float('inf').
    """

    if search_blacklist is None:
        search_blacklist = []

    if any(not isinstance(blacklist_item, str) for blacklist_item in search_blacklist):
        raise TypeError("All items in the search_blacklist must be strings.")

    dir_items_blacklisted = [item for item in os.listdir(directory) if item not in search_blacklist]

    if not dir_items_blacklisted:
        return None, float('inf')

    oldest_item = min(dir_items_blacklisted, key=lambda item: os.path.getctime(os.path.join(directory, item)))

    return oldest_item, os.path.getctime(os.path.join(directory, oldest_item))
