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
    print(f"Invalid estimator name: {name}.")
    return None


def quaternion_normalize(q):
    q = np.array(q, dtype=float)
    return q / np.linalg.norm(q)


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([x, y, z, w])


def quaternion_to_rotation_matrix(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < np.finfo(float).eps:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    R = np.array(
        [
            [1 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1 - (xx + yy)],
        ]
    )
    return R


def quaternion_to_euler(q):
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw
