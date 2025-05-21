# Developer notes

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
ros2 launch microstrain_inertial_driver microstrain_launch.py params_file:=config/imu_params.yml &
ros2 run optical_sensor start_optical.py &
ros2 run estimation simple_estimation.py &
ros2 run server start_server_dash.py
```

## Performance monitoring

```bash
ros2 launch rpi_hw_monitor hw_monitor_launch.py &
ros2 topic echo /hw_status
```

## Record and playback data

Record to a ROS bag:

```bash
ros2 launch main main_launch.py &
ros2 topic list
ros2 bag record /imu/data /imu/mag /optical -o recorded_data
```

Playback with:

```bash
ros2 launch main no_sensors_launch.py &
ros2 bag play recorded_data
```
