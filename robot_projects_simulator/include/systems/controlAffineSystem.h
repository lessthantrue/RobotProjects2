#pragma once

#include <eigen3/Eigen/Eigen>
#include <map>
#include <string>
#include <geometry_msgs/msg/transform.hpp>

// represents a dynamic system of the form:
// dx/dt = f(x) + g(x)u
// f :: R(n) -> R(n)(
// g :: R(n) -> R(n*m)
// u :: R(m)

using std::string;
using std::map;

class ControlAffineSystem {
protected:
    int _dimX, _dimU;
    map<string, int> stateVectorMap;
public:
    int dimX();
    int dimU();
    Eigen::VectorXd state;
    ControlAffineSystem(Eigen::VectorXd initialState);
    virtual Eigen::MatrixXd g() = 0;
    virtual Eigen::VectorXd f();
    Eigen::VectorXd dxdt(Eigen::VectorXd u);
    void step(Eigen::VectorXd u, double dt);
    double getValueByName(string name);
    virtual geometry_msgs::msg::Transform getTransform() = 0;
};
