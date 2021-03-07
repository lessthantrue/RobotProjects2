#include "simObject.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

SimObject::SimObject(SimObjectConfiguration conf)
    : freq(conf.loopHz), name(conf.name), frameId(conf.frameId) {}   


void SimObject::attach(std::shared_ptr<Node> n){
    timer = rclcpp::create_timer(
        n, 
        n->get_clock(), 
        freq.period(), 
        std::bind(&SimObject::timerCallback, this));

    // timeSteps = milliseconds per update
    timeSteps = (int)(std::chrono::duration_cast<std::chrono::milliseconds>(freq.period()).count()); // ns to ms
    RCLCPP_INFO(n->get_logger(), "%s | time steps : %d ms", name.c_str(), timeSteps);
}

void SimObject::attachTf(std::shared_ptr<tf2_ros::Buffer> buf, Node::SharedPtr node){
    tfBuffer = buf;
}
