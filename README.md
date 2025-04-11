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

## Usage

```bash
# Test IMU
ros2 launch microstrain_inertial_driver microstrain_launch.py params_file:=config/imu_params.yml
# ros2 launch microstrain_inertial_driver microstrain_lifecycle_launch.py params_file:=config/imu_params.yml activate:=true

# Test IMU (Lifecycle)
ros2 launch microstrain_inertial_driver microstrain_lifecycle_launch.py params_file:=config/imu_params.yml activate:=false
ros2 lifecycle nodes
ros2 lifecycle list /microstrain_inertial_driver
ros2 lifecycle set /microstrain_inertial_driver activate
ros2 lifecycle set /microstrain_inertial_driver shutdown


# simple test integration
ros2 launch microstrain_inertial_driver microstrain_launch.py &
ros2 run optical_sensor start_optical.py &
ros2 run estimation simple_estimation.py &
ros2 run server start_server_dash.py
```
