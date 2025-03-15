#!/bin/sh

# use conda env if available
if command -v conda >/dev/null 2>&1; then
  source "$(conda info --base)/etc/profile.d/conda.sh"
  if conda env list | grep -q '^\s*estimator-ros\s'; then
    conda activate estimator-ros
  fi
fi

if [ -d /opt/ros/rolling ]; then
    source /opt/ros/rolling/setup.bash
elif [ -d /opt/ros/humble ]; then
    source /opt/ros/humble/setup.bash
elif [ -d ~/ros2_rolling/install ]; then
    source ~/ros2_rolling/install/setup.bash
fi

if [ -d install/ ]; then
    source install/setup.sh
fi

export PYTHONPATH="${CONDA_PREFIX}/lib/python3.12/site-packages:$PYTHONPATH"
# export ROS_DOMAIN_ID=10
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
