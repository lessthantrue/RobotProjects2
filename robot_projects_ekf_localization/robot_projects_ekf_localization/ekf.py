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
        hx = se2.h(self.x, self.beaconPosition)

        thdif = sensed[2] - hx[2]
        if abs(thdif) > np.pi:
            sensed[2] -= (np.sign(thdif) * np.pi * 2)

        self.updateBase(H, hx, sensed, ignoreIndices)

    def updateMultiple(self, sensed, sensorCov, ignoreIndices=[]):
        pass

    def updateBase(self, H, hx, z, ignore=[]):
        for i in ignore:
            H[:,i] = np.zeros(len(z))
            
        y = z - hx
            
        try:
            K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + sensorCov * self.updateCovInflation)
            self.x += K @ y
            self.P = (np.eye(len(self.x)) - K @ H) @ self.P
        except Exception:
            pass
        