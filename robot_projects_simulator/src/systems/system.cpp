#include "systems/system.h"

VectorXd System::noise(VectorXd u){
    return noiseGenerator.samples(1);
}

System::System(Eigen::VectorXd s0) : noiseGenerator(Eigen::VectorXd::Zero(_dimX), Eigen::MatrixXd::Zero(_dimX, _dimX)){
    this->state = s0;
}

void System::step(Eigen::VectorXd u, double dt){
    // clamp input to bounds
    for(int i = 0; i < _dimU; i++){
        u(i) = std::min(std::max(u(i), uLimitLow(i)), uLimitHigh(i));
    }

    // simple euler method for now
    this->state += (this->dxdt(u) + this->noise(u)) * dt;
}

int System::dimX(){
    return _dimX;
}

int System::dimU(){
    return _dimU;
}

double System::getValueByName(string name){
    return state(semanticStateMap[name]);
}

void System::setNoiseCovariance(Eigen::MatrixXd cov){
    this->noiseGenerator.setCovar(cov);
}
