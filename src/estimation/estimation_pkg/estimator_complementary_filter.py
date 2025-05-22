from estimation_pkg.estimator import Estimator
import estimation_pkg.utils as utils
import numpy as np
from estimation.msg import Estimation, Measurements
import rclpy


class ComplementaryFilterEstimator(Estimator):
    def __init__(self, kp: float = 0.5):
        self.kp = kp
        self.reset()

    def reset(self):
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.prev_time = None

        self.p_mouse = np.zeros(3)
        self.prev_mouse_x = None
        self.prev_mouse_y = None

    def update_measurements(self, meas: Measurements):
        t = rclpy.time.Time.from_msg(meas.stamp)
        if self.prev_time is None:
            self.prev_time = t
            self.prev_mouse_x = meas.mouse_integrated_x
            self.prev_mouse_y = meas.mouse_integrated_y
            return

        dt = (t - self.prev_time).nanoseconds * 1e-9
        self.prev_time = t
        if dt <= 0.0:
            return

        a_b = np.array([meas.acceleration.x, meas.acceleration.y, meas.acceleration.z])
        omega_b = np.array(
            [meas.angular_velocity.x, meas.angular_velocity.y, meas.angular_velocity.z]
        )

        R = utils.quaternion_to_rotation_matrix(self.q)
        a_w = R.dot(a_b) - np.array([0.0, 0.0, 9.81])

        self.v += a_w * dt
        self.p += self.v * dt

        omega_q = np.hstack(([0.0], omega_b))
        q_dot = 0.5 * utils.quaternion_multiply(self.q, omega_q)
        self.q += q_dot * dt
        self.q /= np.linalg.norm(self.q)

        dx_m = meas.mouse_integrated_x - self.prev_mouse_x
        dy_m = meas.mouse_integrated_y - self.prev_mouse_y
        self.prev_mouse_x = meas.mouse_integrated_x
        self.prev_mouse_y = meas.mouse_integrated_y

        delta_mouse_body = np.array([dx_m, dy_m, 0.0])
        delta_mouse_world = R.dot(delta_mouse_body)
        self.p_mouse += delta_mouse_world

        self.p = (1.0 - self.kp) * self.p + self.kp * self.p_mouse

        v_mouse = delta_mouse_world / dt
        self.v = (1.0 - self.kp) * self.v + self.kp * v_mouse

    def get_estimation_msg(self) -> Estimation:
        msg = Estimation()
        msg.x = float(self.p[0])
        msg.y = float(self.p[1])
        msg.z = float(self.p[2])
        yaw, pitch, roll = utils.quaternion_to_euler(self.q)
        msg.yaw = float(yaw)
        msg.pitch = float(pitch)
        msg.roll = float(roll)
        return msg
