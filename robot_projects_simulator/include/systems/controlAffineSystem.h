#ifndef CONTROL_AFFINE_SYSTEM_H
#define CONTROL_AFFINE_SYSTEM_H

#include <eigen3/Eigen/Eigen>
#include <map>
#include <string>
#include <geometry_msgs/msg/transform.hpp>
#include "systems/system.h"

// represents a dynamic system of the form:
// dx/dt = f(x) + g(x)u
// f :: R(n) -> R(n)(
// g :: R(n) -> R(n*m)
// u :: R(m)

using std::string;
using std::map;

class ControlAffineSystem : public System {
public:
    ControlAffineSystem(Eigen::VectorXd initialState);
    virtual Eigen::MatrixXd g() = 0;
    virtual Eigen::VectorXd f();
    Eigen::VectorXd dxdt(Eigen::VectorXd u) override;
};

#endif