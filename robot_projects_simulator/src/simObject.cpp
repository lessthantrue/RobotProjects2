#include "simObject.h"
#include <eigen3/Eigen/Dense>
#include <strstream>
#include <iostream>

using namespace Eigen; // too lazy to do it the other way

SimObject::SimObject(std::string _name, IPublisher * viz, ControlAffineSystem * dyn, Duration _period)
    : period(_period), name(_name) {
        visualizer = viz;
        dynamics = dyn;
        lastCtrl = Eigen::VectorXd::Zero(dynamics->dimU());
}

SimObject::SimObject(SimObjectConfiguration conf, IPublisher * viz, ControlAffineSystem * dyn)
    : period(Duration((1.0 / conf.loopHz) * 1000 * 1000 * 1000)), // seconds to nanoseconds 
      name(conf.name), visualizer(viz), dynamics(dyn) {
    lastCtrl = Eigen::VectorXd::Zero(dynamics->dimU());
    visualizer->setFrameId(conf.frameId);
    tfBase.child_frame_id = conf.frameId;
    tfBase.header.frame_id = conf.parentFrame;
}   

void SimObject::controlCallback(const Float64MultiArray::SharedPtr msg){
    lastCtrl = Eigen::Map<Eigen::VectorXd>(&msg->data[0], dynamics->dimU());
}

void SimObject::stepTime(double dt){
    dynamics->step(lastCtrl, dt);
}

void SimObject::visualize(){
    visualizer->setMessage(dynamics);
    visualizer->publish();
}

void SimObject::publishTransform(){
    tfBase.transform = dynamics->getTransform();
    tfPublisher->sendTransform(tfBase);
}

void SimObject::timerCallback(){
    for(int i = 0; i < timeSteps; i++){
        stepTime(0.001); // 1 millisecond
    }
    visualize();
    publishTransform();
}

void SimObject::attach(std::shared_ptr<Node> n){
    controlSub = n->create_subscription<Float64MultiArray>(name + "/control", 10, std::bind(&SimObject::controlCallback, this, _1));
    timer = rclcpp::create_timer(
        n, 
        n->get_clock(), 
        period, 
        std::bind(&SimObject::timerCallback, this));

    // timeSteps = milliseconds per update
    timeSteps = (int)(period.seconds() * 1000);
    RCLCPP_INFO(n->get_logger(), "%s | time steps : %d ms (actual seconds: %f)", name.c_str(), timeSteps, period.seconds());

    visualizer->attach(n, name + "/visualizer");
}

void SimObject::attachTf(std::shared_ptr<tf2_ros::Buffer> buf, Node::SharedPtr node){
    tfListener = std::make_shared<tf2_ros::TransformListener>(*buf);
    tfPublisher = std::make_shared<tf2_ros::TransformBroadcaster>(node);
}
