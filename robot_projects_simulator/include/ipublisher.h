#ifndef IPUBLISHER_H
#define IPUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <string>
#include "systems/controlAffineSystem.h"

using std::string;
using namespace rclcpp;

// rename to something like 'IStatePublisher' in the future
class IPublisher{
public:
    virtual void publish() = 0;
    virtual void attach(std::shared_ptr<Node>, string) = 0;
    virtual void setMessage(ControlAffineSystem *) = 0;
};

#endif