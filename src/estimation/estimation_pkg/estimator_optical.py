from estimation_pkg.estimator import Estimator
import estimation_pkg.utils as utils
import numpy as np
from estimation.msg import Estimation, Measurements
import rclpy


class OpticalEstimator(Estimator):
    def __init__(self):
        self.reset()

    def reset(self):
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.prev_time = None
        self.last_mouse_integrated_x = 0.0
        self.last_mouse_integrated_y = 0.0

    def update_measurements(self, meas: Measurements):
        current_time = rclpy.time.Time.from_msg(meas.stamp)
        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        dx = meas.mouse_integrated_x - self.last_mouse_integrated_x
        dy = meas.mouse_integrated_y - self.last_mouse_integrated_y

        self.last_mouse_integrated_x = meas.mouse_integrated_x
        self.last_mouse_integrated_y = meas.mouse_integrated_y

        self.p[0] += dx
        self.p[1] += dy

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
