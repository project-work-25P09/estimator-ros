from estimation_pkg.estimator import Estimator
import estimation_pkg.utils as utils
import numpy as np
from estimation.msg import Estimation, Measurements
import rclpy


class ComplementaryFilterDetectLiftEstimator(Estimator):
    def __init__(
        self,
        kp: float = 0.5,
        optical_move_thresh: float = 1e-4,
        imu_move_thresh: float = 0.5,
    ):
        self.kp = kp
        self.optical_move_thresh = optical_move_thresh
        self.imu_move_thresh = imu_move_thresh
        self.reset()

    def reset(self):
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.prev_time = None

        self.p_mouse = np.zeros(3)
        self.prev_mouse_x = None
        self.prev_mouse_y = None

        self.optical_valid = True

    def update_measurements(self, meas: Measurements):
        t = rclpy.time.Time.from_msg(meas.header.stamp)
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

        R = utils.quaternion_to_mat(self.q)
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
        delta_mouse_mag = np.linalg.norm(delta_mouse_body)
        delta_mouse_world = R.dot(delta_mouse_body)
        self.p_mouse += delta_mouse_world

        imu_acc_norm = np.linalg.norm(a_w)
        if (
            delta_mouse_mag < self.optical_move_thresh
            and imu_acc_norm > self.imu_move_thresh
        ):
            self.optical_valid = False
        elif delta_mouse_mag >= self.optical_move_thresh:
            self.optical_valid = True

        kp_eff = self.kp if self.optical_valid else 0.0

        self.p = (1.0 - kp_eff) * self.p + kp_eff * self.p_mouse

        v_mouse = delta_mouse_world / dt if dt > 0 else np.zeros(3)
        self.v = (1.0 - kp_eff) * self.v + kp_eff * v_mouse

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
