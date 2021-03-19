#include "systems/system.h"

System::System(Eigen::VectorXd s0){
    this->state = s0;
}

void System::step(Eigen::VectorXd u, double dt){
    // clamp input to bounds
    for(int i = 0; i < _dimU; i++){
        u(i) = std::min(std::max(u(i), uLimitLow(i)), uLimitHigh(i));
    }

    // simple euler method for now
    this->state += this->dxdt(u) * dt;
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
