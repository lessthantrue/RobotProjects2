#ifndef SIM_OBJECT_H
#define SIM_OBJECT_H

#include "ipublisher.h"
#include "systems/controlAffineSystem.h"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen3/Eigen/Eigen>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;
using namespace std_msgs::msg;
using namespace rclcpp;
using std::string;

class SimObjectConfiguration {
public:
    string name;
    string parentFrameId;
    string frameId;
    int loopHz;
};

class SimObject { 
private:
    Rate freq;
    TimerBase::SharedPtr timer;
protected:
    rclcpp::Clock::SharedPtr clock;
    string name, frameId, parentFrameId;
    int timeSteps;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    virtual void timerCallback() = 0;
    virtual void visualize(){}
public:
    typedef std::shared_ptr<SimObject> SharedPtr;
    SimObject(SimObjectConfiguration);
    virtual void attach(std::shared_ptr<Node> n);
    virtual void attachTf(std::shared_ptr<tf2_ros::Buffer>, Node::SharedPtr);
};

#endif