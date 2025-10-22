#!/bin/bash

# Check for argument
if [ -z "$1" ]; then
    echo "Usage: $0 {start|stop}"
    exit 1
fi

case "$1" in
    start)
        echo "Sourcing ROS 2 environment..."
        source /opt/ros/jazzy/setup.bash

        echo "Sourcing workspace environment..."
        source install/setup.bash

        # Create or overwrite the log file
        > run_logs

        echo "Starting receivejson.py in the background..."
        python3 receivejson.py >> run_logs 2>&1 &
        pid1=$!

        echo "Starting processamento_node in the background..."sud
        ros2 run communication_pkg processamento_node >> run_logs 2>&1 &
        pid2=$!

        echo "Starting hand_controller.py in the background..."
        ros2 run controller_pkg hand_controller.py >> run_logs 2>&1 &
        pid3=$!

        echo "All processes started in the background."
        echo "PIDs: $pid1, $pid2, $pid3"

        # Store the PIDs in a file
        echo $pid1 > pids.txt
        echo $pid2 >> pids.txt
        echo $pid3 >> pids.txt
        ;;
    stop)
        echo "Stopping processes..."

        # Read PIDs from the file and kill the processes
        if [ -f pids.txt ]; then
            while read pid; do
                if ps -p $pid > /dev/null; then
                    echo "Killing process with PID: $pid"
                    kill $pid
                else
                    echo "Process with PID: $pid not found."
                fi
            done < pids.txt

            # Remove the PID file
            rm pids.txt
            echo "All processes stopped."
        else
            echo "pids.txt not found. Are the processes running?"
        fi
        ;;
    *)
        echo "Usage: $0 {start|stop}"
        exit 1
        ;;
esac

exit 0
