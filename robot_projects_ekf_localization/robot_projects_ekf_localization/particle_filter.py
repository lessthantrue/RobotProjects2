import numpy as np
import random
from robot_projects_ekf_localization.filter_base import FilterBase
import robot_projects_ekf_localization.se2_dynamics as se2
from geometry_msgs.msg import PoseArray

def lowVarianceSample(elems, weights, count):
    mInv = sum(weights) / count
    r = random.uniform(0, mInv)
    index = 0
    currentWeight = weights[0]
    toReturn = []
    for m in range(0, count):
        U = r + m * mInv
        while U > currentWeight:
            index += 1
            currentWeight += weights[index]
        toReturn.append(elems[index])
    return toReturn
    
class ParticleFilter(FilterBase):
    def __init__(self):
        self.particles = []
        self.numParticles = 100
        self.x = np.zeros(3)
        self.P = np.eye(3)
        self.can_visualize = True
        
    def regenerateParticles(self):
        self.particles = np.random.multivariate_normal(self.x, self.P, size=self.numParticles).tolist()
        assert(len(self.particles) == self.numParticles)
        
    def setInitialPose(self, initialPose):
        self.x = initialPose
        self.regenerateParticles()
        
    def setInitialCovariance(self, initialCovariance):
        self.P = initialCovariance
        self.regenerateParticles()

    def calculateMeanAndCovariance(self):
        self.P = np.cov(np.array(self.particles), rowvar=False)
        self.x = np.mean(self.particles, axis=0)
        
    def predict(self, control, processCovariance, dt):
        processNoises = np.random.multivariate_normal(np.zeros(len(self.x)), processCovariance, size=self.numParticles)
        for i in range(len(self.particles)):
            self.particles[i] = se2.f(self.particles[i], control, dt) + processNoises[i]
        self.calculateMeanAndCovariance()
            
    def update(self, sensed, sensorCov, ignoreIndices=[]):
        # weight step
        w = np.zeros(self.numParticles)
        invSensorCov = np.linalg.inv(sensorCov)
        for i in range(self.numParticles):
            pi = self.particles[i]
            z = se2.h(pi, self.beaconPosition)
            e = sensed - z
            
            if abs(e[2]) > np.pi:
                e[2] = -(np.sign(e[2]) * np.pi * 2 - e[2])
            # set error of ignore indices to zero - I think this works because
            # after this, the contribution to each particle's weight by the 
            # ignored indices will all be equal, so it will have no effect
            # on the outcome of resampling
            # for j in ignoreIndices:
            #     e[j] = 0
                
            w[i] = np.exp(-0.5 * (e.T @ invSensorCov @ e))
            
            
        w /= sum(w) # normalize
            
        # resample step
        self.particles = lowVarianceSample(self.particles, w, self.numParticles)
        
        self.calculateMeanAndCovariance()
        
    def getVisualizationType(self):
        return PoseArray
        
    def getVisualizationData(self):
        poses = PoseArray()
        poses.poses = [ se2.toPose(p) for p in self.particles ]
        return poses