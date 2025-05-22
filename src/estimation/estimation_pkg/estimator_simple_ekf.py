from estimation_pkg.estimator import Estimator
import estimation_pkg.utils as utils
import numpy as np
from estimation.msg import Estimation, Measurements


class SimpleEKF(Estimator):
    def __init__(self, dt, Q, R_imu, R_opt, m_ref):
        self.dt = dt
        self.Q = Q
        self.R_imu = R_imu
        self.R_opt = R_opt
        self.m_ref = m_ref

        self.prev_opt_x = 0
        self.prev_opt_y = 0
        self.reset()

    def reset(self):
        # self.prev_opt_x = 0
        # self.prev_opt_y = 0

        # State
        self.p = np.zeros(3)  # position (x, y, z)
        self.v = np.zeros(3)  # velocity (vx, vy, vz)
        self.q = np.array([0.0, 0.0, 0.0, 1.0])  # orientation quaternion (x, y, z, w)
        self.b_a = np.zeros(3)  # accelerometer bias
        self.b_g = np.zeros(3)  # gyro bias

        # Covariance
        self.P = np.eye(15) * 1e-3

    def get_estimation_msg(self) -> Estimation:
        msg = Estimation()
        msg.x = self.p[0]
        msg.y = self.p[1]
        msg.z = self.p[2]
        msg.yaw, msg.pitch, msg.roll = utils.quaternion_to_euler(self.q)
        return msg

    # def predict(self, meas: Measurements):
    def update_measurements(self, meas: Measurements):
        a = meas.acceleration
        w = meas.angular_velocity
        o = meas.est_orientation
        
        a = np.array([a.x, a.y, a.z])
        w = np.array([w.x, w.y, w.z])
        o = np.array([o.x, o.y, o.z, o.w])

        self.update_optical(meas.mouse_integrated_x, meas.mouse_integrated_y)

        # Remove biases
        a_corr = a - self.b_a
        w_corr = w - self.b_g

        # Attitude propagation via gyro integration
        omega = w_corr * self.dt
        delta_q = np.concatenate([omega * 0.5, [1.0]])
        self.q = utils.quaternion_normalize(utils.quaternion_multiply(self.q, delta_q))

        # Optionally fuse orientation measurement
        if o is not None:
            self.q = utils.quaternion_normalize(o)

        # Transform acceleration to inertial frame and remove gravity
        R = utils.quaternion_to_rotation_matrix(self.q)
        g = np.array([0.0, 0.0, 9.81])
        a_inertial = R.dot(a_corr) - g

        # State propagation: velocity and position
        self.v += a_inertial * self.dt
        self.p += self.v * self.dt + 0.5 * a_inertial * self.dt**2

        # Covariance propagation (simple discrete integration)
        self.P += self.Q

    def update_optical(self, flow_x, flow_y):
        flow_x2 = flow_x - self.prev_opt_x
        flow_y2 = flow_y - self.prev_opt_y
        self.prev_opt_x = flow_x
        self.prev_opt_y = flow_y
        flow_x = flow_x2
        flow_y = flow_y2

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
        delta_q = utils.quaternion_normalize(np.concatenate([phi * 0.5, [1.0]]))
        self.q = utils.quaternion_multiply(delta_q, self.q)
        # Bias corrections
        self.b_a += dx[9:12]
        self.b_g += dx[12:15]

        # Covariance update
        I_KH = np.eye(15) - K.dot(H)
        self.P = I_KH.dot(self.P)
