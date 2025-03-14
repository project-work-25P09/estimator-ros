# Trajectory Estimation Device

This repository contains the ROS2 workspace for the trajectory estimation device built for the Project Work course.

## Installation

Please follow the [setup instructions](doc/SETUP.md).

## Dependencies

```bash
# conda (recommended)
conda env create -f conda/environment.yml
conda activate estimator-ros

# TODO
# https://github.com/ros-drivers/microstrain_mips/tree/master
```

## Cloning and building

```bash
git clone --recursive https://github.com/project-work-25P09/estimator-ros estimator-ros
cd estimator-ros

. source.sh
colcon build --symlink-install
```

## Usage

```bash
```
