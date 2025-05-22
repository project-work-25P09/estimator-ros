from abc import abstractmethod
from estimation.msg import Estimation, Measurements


class Estimator:
    def __init__(self):
        pass

    @abstractmethod
    def get_estimation_msg(self) -> Estimation: ...
    @abstractmethod
    def update_measurements(self, measurements: Measurements): ...
