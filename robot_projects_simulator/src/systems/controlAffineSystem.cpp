#include "systems/controlAffineSystem.h"
#include <iostream>

ControlAffineSystem::ControlAffineSystem(Eigen::VectorXd s0){
    this->state = s0;
}

Eigen::VectorXd ControlAffineSystem::f(){
    return Eigen::VectorXd::Zero(_dimX);
}

Eigen::VectorXd ControlAffineSystem::dxdt(Eigen::VectorXd u){
    return this->f() + this->g() * u;
}

void ControlAffineSystem::step(Eigen::VectorXd u, double dt){
    // simple euler method for now
    this->state += this->dxdt(u) * dt;
}

int ControlAffineSystem::dimX(){
    return _dimX;
}

int ControlAffineSystem::dimU(){
    return _dimU;
}

double ControlAffineSystem::getValueByName(string name){
    return state(stateVectorMap[name]);
}
