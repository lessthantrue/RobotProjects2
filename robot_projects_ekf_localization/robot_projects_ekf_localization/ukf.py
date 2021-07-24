import numpy as np
from robot_projects_ekf_localization.filter_base import FilterBase
import robot_projects_ekf_localization.se2_dynamics as se2
from geometry_msgs.msg import PoseArray

w0 = 0.6

class UnscentedKalmanFilter(FilterBase):
    def __init__(self):
        self.can_visualize = True
    
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
        s = self.getSigmaPoints()
        zs = [ se2.h(si, self.beaconPosition) for si in s ]
        ws = self.getWeights()

        zhat = sum(map(lambda x : x[0] * x[1], zip(zs, ws)))
        shat = sum(map(lambda x : np.outer(x[1] - zhat, x[1] - zhat) * x[0], zip(ws, zs))) + sensorCov
        csz = sum(map(lambda x : np.outer(x[1] - self.x, x[2] - zhat) * x[0], zip(ws, s, zs)))
        K = csz @ np.linalg.inv(shat)

        for i in ignoreIndices:
            K[:, i] = np.zeros(len(sensed))

        self.x += K @ (sensed - zhat)
        # add small identity to matrix to increase numerical stability
        self.P += (np.eye(len(self.x)) * 0.01) - K @ shat @ K.T
        
    def getVisualizationType(self):
        return PoseArray

    def getVisualizationData(self):
        poses = PoseArray()
        poses.poses = [ se2.toPose(s) for s in self.getSigmaPoints() ]
        return poses