import numpy as np
import math

def skew(v):
    return np.array([[    0, -v[2],  v[1]],
                     [ v[2],     0, -v[0]],
                     [-v[1],  v[0],     0]])

def quat_mul(q, r):
    w0,x0,y0,z0 = q; w1,x1,y1,z1 = r
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1
    ])

def quat_to_rot(q):
    w,x,y,z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1-2*(x*x+z*z),   2*(y*z - w*x)],
        [2*(x*z - w*y),   2*(y*z + w*x), 1-2*(x*x+y*y)]
    ])

def small_delta_quat(dtheta):
    half = 0.5 * dtheta
    norm2 = np.dot(half, half)
    return np.concatenate(([1.0], half)) / np.sqrt(1 + norm2)

def quaternion_to_euler(q):
    # returns roll, pitch, yaw
    w,x,y,z = q
    # roll (x-axis rotation)
    sinr = 2*(w*x + y*z)
    cosr = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr, cosr)
    # pitch (y-axis)
    sinp = 2*(w*y - z*x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    # yaw (z-axis)
    siny = 2*(w*z + x*y)
    cosy = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw

class InertialEKF:
    def __init__(self, dt, Q, R_imu, R_optical, m_ref):
        self.dt = dt
        self.Q = Q
        self.R_imu = R_imu      # 3×3 accel noise for attitude
        self.R_opt = R_optical  # 2×2 optical‐flow noise for vel_x, vel_y
        self.m_ref = m_ref

        # State: p(3), v(3), q(4), b_a(3), b_g(3)
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.,0,0,0])
        self.b_a = np.zeros(3)
        self.b_g = np.zeros(3)
        self.P = np.eye(15) * 1e-3

    def predict(self, a_m, w_m):
        a = a_m - self.b_a
        w = w_m - self.b_g
        Rb2n = quat_to_rot(self.q)

        # Nominal propagation
        self.p += self.v * self.dt
        self.v += (Rb2n @ a + np.array([0,0,-9.81])) * self.dt
        dq = small_delta_quat(w * self.dt)
        self.q = quat_mul(dq, self.q)
        self.q /= np.linalg.norm(self.q)

        # Build F_c
        F = np.zeros((15,15))
        F[0:3, 3:6] = np.eye(3)
        F[3:6, 6:9]   = -Rb2n @ skew(a)
        F[3:6, 9:12]  = -Rb2n
        F[6:9, 6:9]   = -skew(w)
        F[6:9, 12:15] = -np.eye(3)

        # Discretize
        Fd = np.eye(15) + F * self.dt
        Qd = self.Q * self.dt
        self.P = Fd @ self.P @ Fd.T + Qd

    def update_acc_mag(self, a_m, m_m):
        # Unused here since mag not subscribed—provided for completeness
        pass

    def update_optical(self, flow_x, flow_y):
        # flow_x, flow_y in m/s
        # H: measure v_x, v_y
        H = np.zeros((2,15))
        H[0,3] = 1.0
        H[1,4] = 1.0

        z = np.array([flow_x, flow_y])
        z_pred = self.v[0:2]
        S = H @ self.P @ H.T + self.R_opt
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ (z - z_pred)

        # Inject
        self.p    += dx[0:3]    # dx[0:3] has zeros except [0],[1]
        self.v    += dx[3:6]
        dq = small_delta_quat(dx[6:9])
        self.q    = quat_mul(dq, self.q)
        self.q   /= np.linalg.norm(self.q)
        self.b_a  += dx[9:12]
        self.b_g  += dx[12:15]
        self.P    = (np.eye(15) - K @ H) @ self.P
