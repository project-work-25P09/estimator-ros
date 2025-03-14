#!/bin/sh

# use conda env if available
if command -v conda >/dev/null 2>&1; then
  source "$(conda info --base)/etc/profile.d/conda.sh"
  if conda env list | grep -q '^\s*estimator-ros\s'; then
    conda activate estimator-ros
  fi
fi

if [ -d /opt/ros/rolling ]; then
    source /opt/ros/rolling/setup.sh
elif [ -d ~/ros2_rolling ]; then
    source ~/ros2_rolling/install/setup.sh
fi

if [ -d install/ ]; then
    source install/setup.sh
fi

export PYTHONPATH="${CONDA_PREFIX}/lib/python3.12/site-packages:$PYTHONPATH"
# export ROS_DOMAIN_ID=10
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
