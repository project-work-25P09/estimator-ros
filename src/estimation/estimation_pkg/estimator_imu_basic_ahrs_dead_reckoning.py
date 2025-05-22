from estimation_pkg.estimator import Estimator
import estimation_pkg.utils as utils
import numpy as np
import rclpy
from estimation.msg import Estimation, Measurements


class BasicAHRSDeadReckoningEstimator(Estimator):
    def __init__(self, alpha: float = 0.02):
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.prev_time = None
        self.alpha = alpha

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
        m_b = np.array(
            [meas.magnetic_field.x, meas.magnetic_field.y, meas.magnetic_field.z]
        )

        # 1. gyro integration
        omega_q = np.hstack(([0.0], omega_b))
        q_dot = 0.5 * utils.quaternion_multiply(self.q, omega_q)
        q_gyro = self.q + q_dot * dt
        q_gyro /= np.linalg.norm(q_gyro)

        # 2. acc + mag estimator
        a_norm = a_b / np.linalg.norm(a_b)
        roll_acc = np.arctan2(a_norm[1], a_norm[2])
        pitch_acc = -np.arcsin(a_norm[0])
        mx, my, mz = m_b
        Xh = mx * np.cos(pitch_acc) + mz * np.sin(pitch_acc)
        Yh = (
            mx * np.sin(roll_acc) * np.sin(pitch_acc)
            + my * np.cos(roll_acc)
            - mz * np.sin(roll_acc) * np.cos(pitch_acc)
        )
        yaw_mag = np.arctan2(-Yh, Xh)
        q_accmag = utils.euler_to_quaternion(yaw_mag, pitch_acc, roll_acc)

        # 3. complementary fusion
        q_fused = (1 - self.alpha) * q_gyro + self.alpha * q_accmag
        q_fused /= np.linalg.norm(q_fused)
        self.q = q_fused

        # 4. velocity and position update
        R = utils.quaternion_to_rotation_matrix(self.q)
        a_w = R.dot(a_b) - np.array([0.0, 0.0, 9.81])
        self.v += a_w * dt
        self.p += self.v * dt

    def get_estimation_msg(self) -> Estimation:
        msg = Estimation()
        msg.x, msg.y, msg.z = map(float, self.p)
        yaw, pitch, roll = utils.quaternion_to_euler(self.q)
        msg.yaw = float(yaw)
        msg.pitch = float(pitch)
        msg.roll = float(roll)
        return msg
