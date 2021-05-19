import numpy as np
from geometry_msgs.msg import PoseWithCovariance
from transforms3d.euler import euler2quat
import robot_projects_ekf_localization.se2_dynamics as se2
from robot_projects_ekf_localization.filter_base import FilterBase

class ExtendedKalmanFilter(FilterBase) :
    def predict(self, control, processCovariance, dt):
        F = se2.F(self.x, control, dt)

        x, y, h = self.x
        nonlinearity = abs(control[0] * h * dt) / 4 * np.sqrt((x * np.cos(h))**2 + (y * np.sin(h))**2)

        self.x = se2.f(self.x, control, dt)
        self.P = F @ self.P @ F.T + processCovariance * (self.predictCovInflation + self.nonlinearCompensation * nonlinearity)

    def update(self, sensed, sensorCov, ignoreIndices=[]):
        H = se2.H(self.x, self.beaconPosition)

        for i in ignoreIndices:
            H[:,i] = np.zeros(len(sensed))
            
        y = sensed - se2.h(self.x, self.beaconPosition)
        if abs(y[2]) > np.pi:
            y[2] = -(np.sign(y[2]) * np.pi * 2 - y[2])

        try:
            K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + sensorCov * self.updateCovInflation)
            self.x += K @ y
            self.P = (np.eye(3) - K @ H) @ self.P
        except Exception:
            pass
