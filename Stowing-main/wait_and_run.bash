#!/bin/bash

# Array of PIDs
pids=(11115 11267 22379)

# Function to check if a process is running
function is_process_running {
    local pid=$1
    if ps -p $pid > /dev/null; then
        return 0  # True: process is running
    else
        return 1  # False: process is not running
    fi
}

# Wait for any one of the jobs to finish
for pid in ${pids[@]}; do
    while is_process_running $pid; do
        sleep 1  # Sleep for a second, then check again
    done
    echo "Process $pid exited."
    break
done

# Run your new job
bash dynamics/scripts/run_train.sh
