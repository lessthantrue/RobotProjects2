#ifndef DYNAMIC_SIM_OBJECT_H
#define DYNAMIC_SIM_OBJECT_H

#include "ipublisher.h"
#include "simObject.h"
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

class DynamicSimObjectConfiguration : public SimObjectConfiguration {
public:
    string parentFrame;
};

class DynamicSimObject : public SimObject { 
private:
    Eigen::VectorXd lastCtrl;
    geometry_msgs::msg::TransformStamped tfBase;
    void controlCallback(const Float64MultiArray::SharedPtr msg);
    void stepTime(double dt);
protected:
    IPublisher * visualizer;
    ControlAffineSystem * dynamics;
    Subscription<Float64MultiArray>::SharedPtr controlSub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfPublisher;
    virtual void timerCallback() override;
    virtual void visualize() override;
    void publishTransform();
public:
    DynamicSimObject(DynamicSimObjectConfiguration &, IPublisher *, ControlAffineSystem *);
    void attach(std::shared_ptr<Node> n) override;
    void attachTf(std::shared_ptr<tf2_ros::Buffer>, Node::SharedPtr) override;
};

#endif