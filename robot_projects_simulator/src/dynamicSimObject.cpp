#include "dynamicSimObject.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace Eigen; // too lazy to do it the other way

DynamicSimObject::DynamicSimObject(DynamicSimObjectConfiguration & conf, IPublisher * viz, ControlAffineSystem * dyn)
    : SimObject(conf), visualizer(viz), dynamics(dyn) {
    lastCtrl = Eigen::VectorXd::Zero(dynamics->dimU());
    visualizer->setFrameId(parentFrameId);
    tfBase.child_frame_id = frameId;
    tfBase.header.frame_id = parentFrameId;
}   

void DynamicSimObject::stepTime(double dt){
    dynamics->step(ctrl->get(), dt);
}

void DynamicSimObject::visualize(){
    visualizer->setMessage(dynamics);
    visualizer->publish();
}

void DynamicSimObject::publishTransform(){
    tfBase.transform = dynamics->getTransform();
    tfBase.header.stamp = clock->now();
    tfPublisher->sendTransform(tfBase);
}

void DynamicSimObject::timerCallback(){
    for(int i = 0; i < timeSteps; i++){
        stepTime(0.001); // 1 millisecond
    }
    visualize();
    publishTransform();
}

void DynamicSimObject::attach(std::shared_ptr<Node> n){
    SimObject::attach(n);
    visualizer->attach(n, name + "/visualizer");
}

void DynamicSimObject::attachTf(std::shared_ptr<tf2_ros::Buffer> buf, Node::SharedPtr node){
    SimObject::attachTf(buf, node);
    tfPublisher = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    tfPublisher->sendTransform(tfBase); // just to prevent lookup errors
}

void DynamicSimObject::setControl(std::shared_ptr<IControl> control){
    this->ctrl = control;
}