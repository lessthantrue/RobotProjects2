#include "simObject.h"
#include <eigen3/Eigen/Dense>

SimObject::SimObject(SimObjectConfiguration conf)
    : freq(conf.loopHz), name(conf.name), 
    frameId(conf.frameId), parentFrameId(conf.parentFrameId) { }   

void SimObject::attach(std::shared_ptr<Node> n){
    clock = n->get_clock();
    timer = rclcpp::create_timer(
        n, 
        clock, 
        freq.period(), 
        std::bind(&SimObject::timerCallback, this));

    // timeSteps = milliseconds per update
    timeSteps = (int)(std::chrono::duration_cast<std::chrono::milliseconds>(freq.period()).count()); // ns to ms
    RCLCPP_INFO(n->get_logger(), "%s\t|\ttime step : %d ms", name.c_str(), timeSteps);
}

void SimObject::attachTf(std::shared_ptr<tf2_ros::Buffer> buf, Node::SharedPtr node){
    tfBuffer = buf;
}
