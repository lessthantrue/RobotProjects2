import numpy as np
from geometry_msgs.msg import PoseWithCovariance
from transforms3d.euler import euler2quat
from math import remainder

def rotation(h):
    return np.array([
        [np.cos(h), -np.sin(h)],
        [np.sin(h), np.cos(h)]
    ])

class ExtendedKalmanFilter :
    def __init__(self):
        self.beaconPosition = np.array([0, 0])

    def setInitialPose(self, initialPose):
        self.x = initialPose

    def setInitialCovariance(self, initialCovariance):
        self.P = initialCovariance

    def setBeaconPosition(self, posn):
        self.beaconPosition = posn

    def toPoseWithCovariance(self):
        pwc = PoseWithCovariance()
        pwc.pose.position.x = self.x[0]
        pwc.pose.position.y = self.x[1]
        # pwc.pose.position.z = float(0)

        q = euler2quat(0, 0, self.x[2])
        pwc.pose.orientation.x = q[1]
        pwc.pose.orientation.y = q[2]
        pwc.pose.orientation.z = q[3]
        pwc.pose.orientation.w = q[0]

        # fill covariance
        # I should make a function for this stuff someday
        pwc.covariance[0] = self.P[0][0]
        pwc.covariance[1] = self.P[0][1]
        pwc.covariance[5] = self.P[0][2]
        pwc.covariance[6] = self.P[1][0]
        pwc.covariance[7] = self.P[1][1]
        pwc.covariance[11] = self.P[1][2]
        pwc.covariance[30] = self.P[2][0]
        pwc.covariance[31] = self.P[2][1]
        pwc.covariance[35] = self.P[2][2]

        return pwc

    # control is [linear velocity, angular velocity]
    def f(self, control, dt):
        h = self.x[2]
        return np.array([
            self.x[0] + control[0] * np.cos(h) * dt,
            self.x[1] + control[0] * np.sin(h) * dt,
            h + control[1] * dt
        ])

    # df/dx
    def F(self, control, dt):
        h = self.x[2]
        return np.array([
            [1, 0, -control[0] * np.sin(h) * dt],
            [0, 1, control[0] * np.cos(h) * dt],
            [0, 0, 1]
        ])

    def h(self):
        dif = self.beaconPosition - self.x[0:2]
        h = self.x[2]
        return np.concatenate([rotation(-h) @ dif, [remainder(h, np.pi * 2)]])

    def H(self):
        dif = self.beaconPosition - self.x[0:2]
        h = self.x[2]
        drotdh = np.array([
            [-np.sin(-h), -np.cos(-h)],
            [np.cos(-h), -np.sin(-h)]
        ])
        return np.block([
            [-rotation(-h), np.reshape(-(drotdh @ dif), (2, 1))],
            [0, 0, 1]
        ])

    def predict(self, control, processCovariance, dt):
        self.x = self.f(control, dt)
        F = self.F(control, dt)
        self.P = F @ self.P @ F.T + processCovariance

    def update(self, sensed, sensorCov, ignoreIndices=[]):
        H = self.H()

        for i in ignoreIndices:
            H[:,i] = np.zeros(len(sensed))
            
        y = sensed - self.h()

        try:
            K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + sensorCov)
            self.x += K @ y
            self.P = (np.eye(3) - K @ H) @ self.P
        except Exception:
            pass
