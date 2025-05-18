# Trajectory Estimation Device

This repository contains the ROS2 workspace for the trajectory estimation device built for the Project Work course.

## Installation

Please follow the [setup instructions](doc/SETUP.md). After the setup, the repository can be built.

```bash
git clone --recursive https://github.com/project-work-25P09/estimator-ros estimator-ros
cd estimator-ros

# conda (recommended)
conda env create -f conda/environment.yml

. source.sh
rosdep install --from-paths src/ -i -r -y
colcon build --symlink-install
```

## Calibration

Modify the configuration files `config/calibration.yml` and `config/estimation.yml` with estimated values.

```bash
# TODO
```

## Usage

To start everything, use the monolithic start script:

```bash
ros2 launch main main_launch.py
```
