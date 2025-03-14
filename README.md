# Trajectory Estimation Device

This repository contains the ROS2 workspace for the trajectory estimation device built for the Project Work course.

## Installation

Please follow the [setup instructions](doc/SETUP.md).

## Cloning and building

```bash
git clone --recursive https://github.com/project-work-25P09/estimator-ros estimator-ros
cd estimator-ros

# conda (recommended)
conda env create -f conda/environment.yml
conda activate estimator-ros

. source.sh
rosdep install --from-paths src/ -i -r -y
colcon build --symlink-install
```

## Usage

```bash
```
