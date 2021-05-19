import numpy as np
from robot_projects_ekf_localization.filter_base import FilterBase
import robot_projects_ekf_localization.se2_dynamics as se2

w0 = 0.7

class UnscentedKalmanFilter(FilterBase):
    def getSigmaPoints(self):
        s = [np.copy(self.x)]
        A = np.linalg.cholesky(self.P)
        L = len(self.x)
        for i in range(L):
            diff = np.sqrt(L / (1 - w0)) * A[:,i]
            s.append(s[0] + diff)
            s.append(s[0] - diff)
        return s

    # for the parameterization used, covariance weights are equal to mean weights
    def getWeights(self):
        w = [w0]
        L = len(self.x)
        wnext = (1 - w0) / (2 * L)
        for _ in range(L):
            w.append(wnext)
            w.append(wnext)
        return w

    def predict(self, control, processCovariance, dt):
        xnew = np.zeros(len(self.x))
        ws = self.getWeights()
        s = self.getSigmaPoints()
        xs = [ se2.f(si, control, dt) for si in s ]

        xnew = sum(map(lambda x : x[0] * x[1], zip(xs, ws)))
        Pnew = sum(map(lambda x : np.outer((x[1] - xnew), (x[1] - xnew).T) * x[0], zip(ws, xs))) + processCovariance

        self.x = xnew
        self.P = Pnew

    def update(self, sensed, sensorCov, ignoreIndices=[]):
        pass