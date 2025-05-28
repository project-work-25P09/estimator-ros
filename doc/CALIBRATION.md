# Calibration

Guide for calibration.

## Optical sensor calibration

Calibrate `x` and `x` directions separately. Set the optical `x_to_m` and `y_to_m` parameters to `1.0` inside the `config/calibration.yml` file. Measure a distance of one meter on a flat table and place pieces of tape at each end. Run the optical sensor and estimation node to start recording, like so:

```bash
ros2 run optical_sensor start_optical.py &
ros2 lifecycle set /optical_sensor_node configure
ros2 lifecycle set /optical_sensor_node activate
ros2 run estimation simple_estimation.py
```

Move the device from one tape to the other. Save the recorded file to a separate folder. Rotate the device by 90° and repeat the test. Save the recorded file again. Move the two files with the names `optical_x.csv` and `optical_y.csv` to the `data/calibration/` folder. Run the following script to estimate the coefficients:

```bash
python3 scripts/calibrate_optical.py
```

Place the estimated coefficients to `calibration.yml`.

## IMU calibration

To calibrate the IMU, first launch the IMU:

```bash
ros2 launch microstrain_inertial_driver microstrain_launch.py params_file:=./config/imu_params.yml &
```

### Gyro bias

```bash
ros2 service call /mip/three_dm/capture_gyro_bias microstrain_inertial_msgs/srv/Mip3dmCaptureGyroBias "{}"

# Save estimated bias to flash memory
ros2 service call /mip/three_dm/device_settings/save std_srvs/srv/Empty "{}"
```

### Magnetic calibration (soft-iron and hard-iron)

```bash
# 1. Record magnetic sweep from all orientations for 2-3 minutes of motion
ros2 bag record /imu/data /imu/data_raw /imu/mag -o data/imu_calibration/magsweep

# 2. Stop IMU launcher

# 3. Dump data to script to CSV and compute matrices
ros2 run calibration dump_mag.py &
ros2 bag play data/imu_calibration/magsweep

# 4. Find calibration parameters
python3 scripts/calibrate_soft_iron_hard_iron.py --flash=no
python3 scripts/calibrate_soft_iron_hard_iron.py --flash=yes
```

### Verify IMU settings

```bash
ros2 service call /mip/base/get_device_information microstrain_inertial_msgs/srv/MipBaseGetDeviceInformation "{}"
```
