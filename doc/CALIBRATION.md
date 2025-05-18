# Calibration

Guide for calibration.

## Optical sensor

Calibrate `x` and `x` directions separately. Set the optical `x_to_m` and `y_to_m` parameters to `1.0` inside the `config/calibration.yml` file. Measure a distance of one meter on a flat table and place pieces of tape at each end. Run the optical sensor and estimation node to start recording, like so:

```bash
ros2 run estimation start_estimation.py
```

Move the device from one tape to the other. Save the recorded file to a separate folder. Rotate the device by 90° and repeat the test. Save the recorded file again. Move the two files with the names `optical_x.csv` and `optical_y.csv` to the `data/calibration/` folder. Run the following script to estimate the coefficients:

```bash
python3 scripts/calibrate_optical.py
```

Place the estimated coefficients to `calibration.yml`.
