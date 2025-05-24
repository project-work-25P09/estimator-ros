import numpy as np
import math
from estimation_pkg.estimator_simple_ekf import SimpleEKF
from estimation_pkg.estimator_imu_dead_reckoning import (
    DeadReckoningEstimator,
)
from estimation_pkg.estimator_imu_basic_ahrs_dead_reckoning import (
    BasicAHRSDeadReckoningEstimator,
)
from estimation_pkg.estimator_complementary_filter import (
    ComplementaryFilterEstimator,
)
from estimation_pkg.estimator_complementary_filter_detect_lift import (
    ComplementaryFilterDetectLiftEstimator,
)
from estimation_pkg.estimator_optical import OpticalEstimator
from estimation_pkg.estimator_optical_imu_integrator import OpticalImuIntegratorEstimator

def get_estimator(name):
    if name == "simple_ekf":
        return SimpleEKF(1.0 / 200.0, np.eye(15), np.eye(3), np.eye(2), [0.0, 0.0, 0.0])
    elif name == "imu_dead_reckoning":
        return DeadReckoningEstimator()
    elif name == "imu_basic_ahrs_dead_reckoning":
        return BasicAHRSDeadReckoningEstimator(alpha=0.02)
    elif name == "complementary_dead_reckoning":
        return ComplementaryFilterEstimator(kp=0.5)
    elif name == "complementary_dead_reckoning_detect_lift":
        return ComplementaryFilterDetectLiftEstimator(
            kp=0.5, optical_move_thresh=1e-4, imu_move_thresh=0.5
        )
    elif name == "optical_dead_reckoning":
        return OpticalEstimator()
    elif name == "optical_imu_integrator":
        return OpticalImuIntegratorEstimator()
    print(f"Invalid estimator name: {name}.")
    return None


def list_available_estimators():
    return [
        "simple_ekf",
        "imu_dead_reckoning",
        "imu_basic_ahrs_dead_reckoning",
        "complementary_dead_reckoning",
        "complementary_dead_reckoning_detect_lift",
        "optical_dead_reckoning",
        "optical_imu_integrator",
    ]
