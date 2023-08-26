#!/bin/bash

# Copyright 2022 Shadow Robot Company Ltd.
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

# Example use: ./environment_setup.sh -b /home/user/.bashrc -c "source /home/user/example_workspace/devel/setup.bash" "export example_var=1"

set -e
tag_number=$RANDOM

while [[ $# > 1 ]]
do
key="$1"

case $key in
    -b|--bashrcpath)
    bashrc_path="$2"
    shift
    ;;
    -c|--commands)
    commands=("${@:2}")
    shift
    ;;
    *)
    # ignore unknown option
    ;;
esac
shift
done

if [ -z ${bashrc_path+x} ]; then bashrc_path='.bashrc'; fi

function check_if_bashrc_ends_with_empty_line {
    if ! [ -z "$(tail -c 1 "$bashrc_path")" ]
    then
        echo -en "\n" >> $bashrc_path
    fi
}

function append_to_bash {
    for item in "${commands[@]}"
    do
        # Do not add to .bashrc if starts with '__'. This is to prevent rosrun arguments to be added.
        if ! [[ "$item" =~ ^__ ]]
        then
            echo -e "$item" "# tmp_$tag_number" >> $bashrc_path
        fi
    done
}

function clean_up {
sed -i "/# tmp_$tag_number$/d" $bashrc_path
}

check_if_bashrc_ends_with_empty_line
append_to_bash
trap clean_up EXIT

echo "Kill this program to revert to previous .bashrc"

while true; do sleep 1; done
