#ifndef SIM_OBJECT_H
#define SIM_OBJECT_H

#include "ipublisher.h"
#include "systems/controlAffineSystem.h"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen3/Eigen/Eigen>

using std::placeholders::_1;
using namespace std_msgs::msg;
using namespace rclcpp;
using std::string;

class SimObject { 
private:
    TimerBase::SharedPtr timer;
    Eigen::VectorXd lastCtrl;
    Duration period;
    void controlCallback(const Float64MultiArray::SharedPtr msg);
protected:
    IPublisher * visualizer;
    ControlAffineSystem * dynamics;
    int timeSteps;
    Subscription<Float64MultiArray>::SharedPtr controlSub;
    string name;
    virtual void timerCallback();
public:
    SimObject(string name, IPublisher *, ControlAffineSystem *, Duration);
    void stepTime(double dt);
    void visualize();
    void attach(std::shared_ptr<Node> n);
};

#endif