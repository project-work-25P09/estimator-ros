#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, MagneticField
from estimation.msg import Estimation
import numpy as np
import math

# -------------------------------------------------------------------
# === Quaternion & EKF implementation ===
# -------------------------------------------------------------------
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

# -------------------------------------------------------------------
# === ROS2 Node ===
# -------------------------------------------------------------------
class EstimatorNode(Node):
    def __init__(self):
        super().__init__('estimator_node')

        # publisher & subscriptions
        self.est_pub = self.create_publisher(Estimation, '/estimation', 10)
        self.create_subscription(Point, '/optical', self.optical_callback, 10)
        self.create_subscription(Imu,   '/imu/data',  self.imu_callback,    200)

        # optical‐to‐meters scale
        self.optical_to_m = 2.0 * 3000.0/19000.0 * 1e-3

        # filter parameters
        dt = 1.0/200.0
        Q = np.eye(15)*1e-4
        R_imu = np.eye(3)*0.5
        R_opt = np.eye(2)*(0.1**2)
        m_ref = np.array([0.,0.,0.])   # unused here

        self.ekf = InertialEKF(dt, Q, R_imu, R_opt, m_ref)
        self.get_logger().info('EstimatorNode started.')

    def imu_callback(self, imu_msg: Imu):
        # build numpy vectors
        a = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z,
        ])
        w = np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z,
        ])

        # propagate IEKF
        self.ekf.predict(a, w)

        # optionally: attitude correction with accel/mag
        # self.ekf.update_acc_mag(a, mag_vector)

        self.publish_estimation()

    def optical_callback(self, opt_msg: Point):
        # convert flow->velocity (m/s)
        flow_x = opt_msg.x * self.optical_to_m
        flow_y = opt_msg.y * self.optical_to_m

        # fuse optical‐flow velocity
        self.ekf.update_optical(flow_x, flow_y)

        self.publish_estimation()

    def publish_estimation(self):
        est = Estimation()
        # position
        est.x, est.y, est.z = self.ekf.p.tolist()
        # angles
        roll, pitch, yaw = quaternion_to_euler(self.ekf.q)
        est.roll, est.pitch, est.yaw = roll, pitch, yaw
        # biases
        est.acc_x, est.acc_y, est.acc_z = self.ekf.b_a.tolist()
        est.mag_x, est.mag_y, est.mag_z = 0.0, 0.0, 0.0
        est.mag_strength = 0.0
        # optical‐flow diagnostics
        est.mouse_movement  = 0.0
        est.mouse_speed     = math.hypot(est.x, est.y)
        est.mouse_direction = math.atan2(est.y, est.x) if est.x or est.y else 0.0
        est.mouse_distance  = math.hypot(est.x, est.y)

        self.est_pub.publish(est)

def main(args=None):
    rclpy.init(args=args)
    node = EstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
