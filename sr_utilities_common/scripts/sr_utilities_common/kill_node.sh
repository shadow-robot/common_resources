#!/bin/bash

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