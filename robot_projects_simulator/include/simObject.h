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
    string frameId;
    string parentFrame;
    int loopHz;
};

class SimObject { 
private:
    Duration period;
    TimerBase::SharedPtr timer;
    Eigen::VectorXd lastCtrl;
    geometry_msgs::msg::TransformStamped tfBase;
    void controlCallback(const Float64MultiArray::SharedPtr msg);
protected:
    string name;
    IPublisher * visualizer;
    ControlAffineSystem * dynamics;
    int timeSteps;
    Subscription<Float64MultiArray>::SharedPtr controlSub;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfPublisher;
    virtual void timerCallback();
    void visualize();
    void publishTransform();
public:
    SimObject(string name, IPublisher *, ControlAffineSystem *, Duration);
    SimObject(SimObjectConfiguration, IPublisher *, ControlAffineSystem *);
    void stepTime(double dt);
    void attach(std::shared_ptr<Node> n);
    void attachTf(std::shared_ptr<tf2_ros::Buffer>, Node::SharedPtr);
};

#endif