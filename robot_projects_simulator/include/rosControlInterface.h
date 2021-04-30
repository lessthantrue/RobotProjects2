#ifndef ROS_CONTROL_INTERFACE_H
#define ROS_CONTROL_INTERFACE_H

#include "IControl.h"
#include <rclcpp/rclcpp.hpp>
#include <map>

using namespace rclcpp;
using std::map;
using namespace std::placeholders;

template <typename T>
class RosControlInterface : public IControl {
private:
    typename Subscription<T>::SharedPtr controlSub;
protected:
    T latest;
public:
    void attach(Node::SharedPtr node, string topicName);
};

// I don't know why this doesn't work.
// Good luck
template<typename T>
void RosControlInterface<T>::attach(Node::SharedPtr node, string topicName){
    auto fn = [=](const std::shared_ptr<T> msg){ latest = *msg; };
    controlSub = node->template create_subscription<T>(topicName, 5, fn);
}

#endif