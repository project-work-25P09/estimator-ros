import numpy as np
import math


def quaternion_normalize(q):
    q = np.array(q, dtype=float)
    return q / np.linalg.norm(q)


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([x, y, z, w])


def quaternion_to_rotation_matrix(q):
    x, y, z, w = q
    # normalize to avoid numerical drift
    n = x*x + y*y + z*z + w*w
    if n < np.finfo(float).eps:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s*w*x, s*w*y, s*w*z
    xx, xy, xz = s*x*x, s*x*y, s*x*z
    yy, yz, zz = s*y*y, s*y*z, s*z*z
    R = np.array([
        [1 - (yy + zz),     xy - wz,          xz + wy],
        [xy + wz,           1 - (xx + zz),    yz - wx],
        [xz - wy,           yz + wx,          1 - (xx + yy)]
    ])
    return R


def quaternion_to_euler(q):
    x, y, z, w = q
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis rotation)
    sinp = 2.0 * (w*y - z*x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi/2, sinp)
    else:
        pitch = math.asin(sinp)
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class InertialEKF:
    def __init__(self, dt, Q, R_imu, R_opt, m_ref):
        self.dt = dt
        self.Q = Q
        self.R_imu = R_imu
        self.R_opt = R_opt
        self.m_ref = m_ref

        # State
        self.p = np.zeros(3)       # position (x, y, z)
        self.v = np.zeros(3)       # velocity (vx, vy, vz)
        self.q = np.array([0.0, 0.0, 0.0, 1.0])  # orientation quaternion (x, y, z, w)
        self.b_a = np.zeros(3)     # accelerometer bias
        self.b_g = np.zeros(3)     # gyro bias

        # Covariance
        self.P = np.eye(15) * 1e-3

    def predict(self, a, w, o=None):
        # Remove biases
        a_corr = a - self.b_a
        w_corr = w - self.b_g

        # Attitude propagation via gyro integration
        omega = w_corr * self.dt
        delta_q = np.concatenate([omega * 0.5, [1.0]])
        self.q = quaternion_normalize(quaternion_multiply(self.q, delta_q))

        # Optionally fuse orientation measurement
        if o is not None:
            self.q = quaternion_normalize(o)

        # Transform acceleration to inertial frame and remove gravity
        R = quaternion_to_rotation_matrix(self.q)
        g = np.array([0.0, 0.0, 9.81])
        a_inertial = R.dot(a_corr) - g

        # State propagation: velocity and position
        self.v += a_inertial * self.dt
        self.p += self.v * self.dt + 0.5 * a_inertial * self.dt**2

        # Covariance propagation (simple discrete integration)
        self.P += self.Q

    def update_optical(self, flow_x, flow_y):
        # Measurement z: velocity from optical flow
        z = np.array([flow_x / self.dt, flow_y / self.dt])

        # Measurement model H: pick out vx, vy from state error vector
        H = np.zeros((2, 15))
        H[0, 3] = 1
        H[1, 4] = 1

        # Innovation y
        v_pred = np.array([self.v[0], self.v[1]])
        y = z - v_pred

        # Innovation covariance S and Kalman gain K
        S = H.dot(self.P).dot(H.T) + self.R_opt
        K = self.P.dot(H.T).dot(np.linalg.inv(S))

        # State correction dx
        dx = K.dot(y)
        # Update state estimates
        self.p += dx[0:3]
        self.v += dx[3:6]
        # Attitude correction
        phi = dx[6:9]
        delta_q = quaternion_normalize(np.concatenate([phi * 0.5, [1.0]]))
        self.q = quaternion_multiply(delta_q, self.q)
        # Bias corrections
        self.b_a += dx[9:12]
        self.b_g += dx[12:15]

        # Covariance update
        I_KH = np.eye(15) - K.dot(H)
        self.P = I_KH.dot(self.P)
