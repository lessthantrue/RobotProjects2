import numpy as np
from geometry_msgs.msg import PoseWithCovariance
from transforms3d.euler import euler2quat
from abc import ABC, abstractmethod

class FilterBase(ABC):
    def __init__(self):
        self.beaconPosition = np.array([0, 0]) 
        self.updateCovInflation = 1
        self.nonlinearCompensation = 0
        self.predictCovInflation = 1
        self.can_visualize = False

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

    @abstractmethod
    def predict(self, control, processCovariance, dt):
        pass

    @abstractmethod
    def update(self, sensed, sensorCov, ignoreIndices = []):
        pass

    def canVisualize(self):
        return self.can_visualize
    
    # type returned must have a header field
    @abstractmethod
    def getVisualizationType(self):
        pass
    
    @abstractmethod
    def getVisualizationData(self):
        pass
    