from estimation_pkg.estimator import Estimator
import estimation_pkg.utils as utils
import numpy as np
from estimation.msg import Estimation, Measurements
import rclpy


class DeadReckoningEstimator(Estimator):
    def __init__(self):
        self.reset()

    def reset(self):
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.prev_time = None

    def update_measurements(self, meas: Measurements):
        current_time = rclpy.time.Time.from_msg(meas.stamp)
        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

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
