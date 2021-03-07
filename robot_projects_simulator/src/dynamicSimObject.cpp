#include "dynamicSimObject.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace Eigen; // too lazy to do it the other way

DynamicSimObject::DynamicSimObject(DynamicSimObjectConfiguration & conf, IPublisher * viz, ControlAffineSystem * dyn)
    : SimObject(conf), visualizer(viz), dynamics(dyn) {
    lastCtrl = Eigen::VectorXd::Zero(dynamics->dimU());
    visualizer->setFrameId(conf.parentFrame);
    tfBase.child_frame_id = conf.frameId;
    tfBase.header.frame_id = conf.parentFrame;
}   

void DynamicSimObject::controlCallback(const Float64MultiArray::SharedPtr msg){
    lastCtrl = Eigen::Map<Eigen::VectorXd>(&msg->data[0], dynamics->dimU());
}

void DynamicSimObject::stepTime(double dt){
    dynamics->step(lastCtrl, dt);
}

void DynamicSimObject::visualize(){
    visualizer->setMessage(dynamics);
    visualizer->publish();
}

void DynamicSimObject::publishTransform(){
    tfBase.transform = dynamics->getTransform();
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
    controlSub = n->create_subscription<Float64MultiArray>(name + "/control", 10, std::bind(&DynamicSimObject::controlCallback, this, _1));
    visualizer->attach(n, name + "/visualizer");
}

void DynamicSimObject::attachTf(std::shared_ptr<tf2_ros::Buffer> buf, Node::SharedPtr node){
    SimObject::attachTf(buf, node);
    tfPublisher = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    tfPublisher->sendTransform(tfBase); // just to prevent lookup errors
}
