#!/bin/sh

# use conda env if available and not already activated
if command -v conda >/dev/null 2>&1; then
  source "$(conda info --base)/etc/profile.d/conda.sh"
  if [ "$CONDA_DEFAULT_ENV" != "estimator-ros" ]; then
    if conda env list | grep -q '^\s*estimator-ros\s'; then
      conda activate estimator-ros
    fi
  fi
fi

if [ -d /opt/ros/rolling ]; then
    source /opt/ros/rolling/setup.bash
elif [ -d /opt/ros/humble ]; then
    source /opt/ros/humble/setup.bash
elif [ -d ~/ros2_rolling/install ]; then
    source ~/ros2_rolling/install/setup.bash
fi

export PYTHONPATH="${CONDA_PREFIX}/lib/python3.10/site-packages"

if [ -d install/ ]; then
    source install/setup.sh
fi
