#include "simulation.h"

using std::string;

Simulation::Simulation(Duration timeStep) 
    : Node("simulation"), defaultTimeStep(timeStep){
    sworld = std::make_shared<SensableWorld>();
    buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    RCLCPP_INFO(this->get_logger(), "Simulation Initialized");
}

void Simulation::addObject(SimObject::SharedPtr obj){
    obj->attach(this->shared_from_this());
    obj->attachTf(buffer, this->shared_from_this());
    RCLCPP_DEBUG(this->get_logger(), "Added new object");
    simObjects.push_back(obj);
}

void Simulation::timeStep(){
    rclcpp::spin_some(this->shared_from_this());
}

std::shared_ptr<SensableWorld> Simulation::sensableWorld(){
    if(!sworld->isAttached()){
        sworld->attach(this->shared_from_this());
    }
    return sworld;
}