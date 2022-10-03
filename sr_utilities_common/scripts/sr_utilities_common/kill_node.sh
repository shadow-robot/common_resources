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


node_name=$1
while true; do
    if [ "$(ps aux | grep -v 'grep' | grep -v 'kill' | grep -c $node_name)" -ne 0 ]; then
        node_pid=$(ps aux | grep "name:=$node_name" | grep -v "grep" | awk '{print $2}')
        restart_command=$(echo $(ps -p $node_pid -o args=))
        echo "Killing desired node! To recover it, run:"
        echo $restart_command
        kill -9 $node_pid
        break
    fi
done