// #include "systems/simpleSE2.h"
#include "systems/simpleSE2.h"
#include <math.h>

SimpleSE2::SimpleSE2(double x, double y, double h)
    : ControlAffineSystem(Eigen::Vector3d(x, y, h)){
    stateVectorMap["x"] = 0;
    stateVectorMap["y"] = 1;
    stateVectorMap["h"] = 2;
    _dimX = 3;
    _dimU = 2;
}

Eigen::MatrixXd SimpleSE2::g() {
    Eigen::MatrixXd gx(dimX(), dimU());
    gx << cos(state(2)), 0,
          sin(state(2)), 0,
          0, 1;
    return gx;
}
