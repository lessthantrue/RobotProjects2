import numpy as np
import math

# common functions for the robot dynamics and sensor model

# rotation matrix
def rotation(h):
    return np.array([
        [np.cos(h), -np.sin(h)],
        [np.sin(h), np.cos(h)]
    ])

# control is [linear velocity, angular velocity]
def f(x, control, dt):
    h = x[2]
    return np.array([
        x[0] + control[0] * np.cos(h) * dt,
        x[1] + control[0] * np.sin(h) * dt,
        h + control[1] * dt
    ])

# df/dx
def F(x, control, dt):
    h = x[2]
    return np.array([
        [1, 0, -control[0] * np.sin(h) * dt],
        [0, 1, control[0] * np.cos(h) * dt],
        [0, 0, 1]
    ])

def h(x, beaconPosition):
    dif = beaconPosition - x[0:2]
    h = x[2]
    return np.concatenate([rotation(-h) @ dif, [math.remainder(h, np.pi * 2)]])

def H(x, beaconPosition):
    dif = beaconPosition - x[0:2]
    h = x[2]
    drotdh = np.array([
        [-np.sin(-h), -np.cos(-h)],
        [np.cos(-h), -np.sin(-h)]
    ])
    return np.block([
        [-rotation(-h), np.reshape(-(drotdh @ dif), (2, 1))],
        [0, 0, 1]
    ])