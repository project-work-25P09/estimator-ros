from abc import abstractmethod
from estimation.msg import Estimation, Measurements

class EKF:
    def __init__(self):
        pass

    @abstractmethod
    def get_estimation_msg(self) -> Estimation: ...

    @abstractmethod
    def predict(self, measurements: Measurements): ...
