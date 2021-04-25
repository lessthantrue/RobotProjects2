import numpy as np
from geometry_msgs.msg import PoseWithCovariance

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
        pwc.pose.position.y = self.x[0]
        # pwc.pose.position.z = float(0)

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
        rot = np.array([
            [np.cos(h), -np.sin(h)],
            [np.sin(h), np.cos(h)]
        ])
        return rot @ dif

    # it's a huge mess
    # g(x) = [ x[0], x[1] ] -> dg/dx(x) = [ [1, 0, 0], [0, 1, 0], ]
    # h(x) = [R(x), 0; 0, 1]*(b-g(x), x[2])
    # dh/dx(x)  = H
    #           = dR/dx(x)*(b-g(x)) + R(x)*d(b-g(x))/dx 
    #           = dR/dx(x)*(b-g(x)) - R(x)*dg/dx(x)
    #           = dR/dx(x)*(b-g(x)) - [R(x):0]
    # dh/dx :: R^(3x3)
    # dR/dx = [ dR/dx0, dR/dx1, dR/dx2 ] = [ 0, 0, dR/dh ]
    def H(self):
        # dif = b - g(x)
        dif = self.beaconPosition - self.x[0:2]
        h = self.x[2]
        # rotation matrix
        R = np.array([
            [np.cos(h), -np.sin(h)],
            [np.sin(h), np.cos(h)]
        ])
        drotdh = np.array([
            [-np.sin(h), -np.cos(h)],
            [np.cos(h), -np.sin(h)]
        ])
        dRdx = np.array([np.zeros((2, 2)), np.zeros((2, 2)), drotdh])
        dhdx = np.tensordot(dRdx, dif) - np.concatenate(R, np.zeros(2))
        return np.block([[dhdx], [np.zeros((2, 1)), 1]]) # I hope this works

    def predict(self, control, processCovariance, dt):
        self.x = self.f(control, dt)
        F = self.F(control, dt)
        self.P = F @ self.P @ F.T + processCovariance

    def update(self, sensed, sensorCov):
        H = self.H()
        y = sensed - self.h()
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + sensorCov)
        self.x += K @ y
        self.P = (np.eye(3) - K @ H) @ self.P