#include "systems/controlAffineSystem.h"
#include <iostream>

ControlAffineSystem::ControlAffineSystem(Eigen::VectorXd s0) : System(s0) {}

Eigen::VectorXd ControlAffineSystem::f(){
    return Eigen::VectorXd::Zero(_dimX);
}

Eigen::VectorXd ControlAffineSystem::dxdt(Eigen::VectorXd u){
    return this->f() + this->g() * u;
}