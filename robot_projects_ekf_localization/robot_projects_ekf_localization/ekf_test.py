from robot_projects_ekf_localization import ekf
import numpy as np

# Verifies that the math used to define H(x) = dh(x)/dx is correct
def test_H():
    # The theory behind this test:
    # dh(x(t))/dt = H(x) @ dx/dt -> h(x + v) = h(x) + H(x) @ v + e for small v, e is error term
    # re-arrange to get e = h(x + v) - h(x) - H(x) @ v
    # so, generate some random vs and xs and take a look at the error patterns 
    # error should be on the order of v

    covV = np.array([
        [0.01, 0, 0],
        [0, 0.01, 0],
        [0, 0, 0.01]
    ])

    covX = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    mean = np.zeros(3)

    ntest = 10000
    errors = [np.zeros(3)] * ntest

    ekf_inst = ekf.ExtendedKalmanFilter()
    ekf_inst.setBeaconPosition(np.array([3, -2]))

    for i in range(ntest):
        v = np.random.multivariate_normal(mean, covV)
        x = np.random.multivariate_normal(mean, covX)
        ekf_inst.x = x + v
        expected = ekf_inst.h()
        ekf_inst.x = x
        errors[i] = expected - ekf_inst.h() - ekf_inst.H() @ v

    # print(np.std(errors))
    # print(np.linalg.norm(sum(errors) / ntest))
    # print(errors)
    print(np.average(list(map(np.linalg.norm, errors))))

test_H()