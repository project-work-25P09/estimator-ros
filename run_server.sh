#!/bin/bash
# filepath: /home/jtammisto/estimator-ros/run_server.sh

# Source the ROS workspace
source /home/jtammisto/estimator-ros/install/setup.bash

# Run the server directly to see more detailed output
python3 /home/jtammisto/estimator-ros/src/server/scripts/start_server.py
